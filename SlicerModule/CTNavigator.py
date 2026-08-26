"""
# References:
# [1] Haxthausen et al., "UltrARsound: in situ visualization of live
#     ultrasound images using HoloLens 2", IJCARS 17, 2081-2091 (2022)
#     https://doi.org/10.1007/s11548-022-02695-z
# [2] BSEL-UC3M, "OpenARHealth", GitHub (2022)
#     https://github.com/BSEL-UC3M/OpenARHealth
# [3] Sevilla Garcia et al., "3D Slicer Module Implementation in Python",
#     BSEL-UC3M / IGT Workshop (2025)

CTNavigator - 3D Slicer Module (v3)
=====================================
Modulo de navegacion quirurgica para CT. El instrumento se coloca en el
CT mediante una cadena nativa de transforms de Slicer, sin recalcular la
pose en Python: Slicer propaga las transforms automaticamente.

Cadena de transformadas (arbol de nodos):

    MarkerToCT              <- resultado del registro (Fiducial Registration
    |                          Wizard). Al arrancar es identidad.
    +-- ToolToMarker        <- llega del C# por OpenIGTLink (pose del
        |                      instrumento en el sistema del marcador).
        +-- Tool (STL)      <- modelo del instrumento (opcional, decorativo).
        +-- TipCT           <- markup de la punta (origen de ToolToMarker).

Con esta cadena, cuando el C# actualiza ToolToMarker el instrumento y la
punta se mueven solos, y cuando el FRW rellena MarkerToCT todo queda
alineado con el CT. El registro es lo que ata el espacio fisico al del TAC.

Estructura del modulo:
  SETUP
    LOAD          -> CT, biomodelo (obligatorios); marcador e instrumento
                     STL (opcionales, solo visualizacion)
    CONNECTION    -> lanza el .exe y conecta por OpenIGTLink
  REGISTRO        -> captura de puntos y calculo via FRW (requiere tracking)
  TOGGLES
    Opacidades y surgeon display
"""

import numpy as np
import vtk
import ctk
import qt
import slicer
from slicer.ScriptedLoadableModule import (
    ScriptedLoadableModule,
    ScriptedLoadableModuleWidget,
    ScriptedLoadableModuleLogic,
    ScriptedLoadableModuleTest,
)


# ─────────────────────────────────────────────────────────────────────────────
# Metadata
# ─────────────────────────────────────────────────────────────────────────────

class CTNavigator(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title        = "Universal Surgical Navigator"
        self.parent.categories   = ["Navigation"]
        self.parent.dependencies = []
        self.parent.contributors = [""]
        self.parent.helpText = (
            "Navegador quirurgico para CT. Coloca el instrumento en el CT "
            "mediante la cadena nativa de transforms MarkerToCT -> ToolToMarker, "
            "donde ToolToMarker llega del tracker por OpenIGTLink y MarkerToCT "
            "es el resultado del registro (Fiducial Registration Wizard)."
        )


# ─────────────────────────────────────────────────────────────────────────────
# Widget
# ─────────────────────────────────────────────────────────────────────────────

class CTNavigatorWidget(ScriptedLoadableModuleWidget):

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        self.logic  = CTNavigatorLogic()

        # La interfaz vive en Resources/UI/CTNavigator.ui, editable con Qt Designer.
        # Aqui solo se carga, se conectan las senales y se hace lo que el .ui no puede:
        # pasar la escena MRML a los combos y rellenar el desplegable de modo.
        uiWidget = slicer.util.loadUI(self.resourcePath('UI/CTNavigator.ui'))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)
        uiWidget.setMRMLScene(slicer.mrmlScene)

        self._secondaryWindow = None   # referencia a la ventana, None = no abierta

        self._setupUIState()
        self._setupTransformChain()
        self._setupRegistrationTables()
        self._connectSignals()

    def _setupRegistrationTables(self):
        """Crea las dos tablas de puntos del registro con qSlicerSimpleMarkupsWidget
        (el mismo widget que usa el FRW). Cada tabla queda fija a su lista y con el
        selector de nodo oculto. Las listas se crean aqui, en la raiz (sin transform
        padre), como exige el FRW.

            RASPoints        -> puntos en el CT, con place mode (se clican en el modelo)
            ReferencePoints  -> puntos reales, se llenan con 'Capture tip point'
        """
        ras = self.logic.getOrCreatePointList("RASPoints", color=(0.2, 0.4, 1.0))
        ref = self.logic.getOrCreatePointList("ReferencePoints", color=(1.0, 0.2, 0.2))

        # El layout no se expone en self.ui (solo los widgets). Se accede a traves
        # del widget contenedor registrationTablesHost, que si es un widget.
        layout = self.ui.registrationTablesHost.layout()

        # --- Tabla CT (RASPoints): con place mode para clicar en el modelo ---
        ctCol = qt.QVBoxLayout()
        ctCol.addWidget(qt.QLabel("CT points (click on model)"))
        self.rasTable = slicer.qSlicerSimpleMarkupsWidget()
        self.rasTable.setMRMLScene(slicer.mrmlScene)
        self.rasTable.setCurrentNode(ras)
        self.rasTable.enterPlaceModeOnNodeChange = False
        self._hideNodeSelector(self.rasTable)
        ctCol.addWidget(self.rasTable)
        layout.addLayout(ctCol)

        # --- Tabla instrumento (ReferencePoints): sin place (se captura con boton) ---
        tipCol = qt.QVBoxLayout()
        tipCol.addWidget(qt.QLabel("Tip points (captured)"))
        self.refTable = slicer.qSlicerSimpleMarkupsWidget()
        self.refTable.setMRMLScene(slicer.mrmlScene)
        self.refTable.setCurrentNode(ref)
        self.refTable.enterPlaceModeOnNodeChange = False
        # Desactivar el place mode de esta tabla: sus puntos vienen de la punta
        try:
            self.refTable.markupsPlaceWidget().placeModeEnabled = False
            self.refTable.markupsPlaceWidget().setPlaceButtonVisibility(False)
        except Exception:
            pass
        self._hideNodeSelector(self.refTable)
        tipCol.addWidget(self.refTable)
        layout.addLayout(tipCol)

    def _hideNodeSelector(self, simpleMarkupsWidget):
        """Oculta el combo de seleccion de nodo del qSlicerSimpleMarkupsWidget,
        para que la tabla quede fija a su lista y el usuario no pueda cambiarla."""
        for childName in ("MarkupsFiducialNodeComboBox", "MarkupsNodeComboBox"):
            found = slicer.util.findChildren(simpleMarkupsWidget, childName)
            for w in found:
                w.setVisible(False)

    def _setupTransformChain(self):
        """Crea la cadena de transforms vacia para que exista desde el arranque:

            MarkerToCT (identidad)
            +-- ToolToMarker (lo rellena el C#)
                +-- TipCT (punta)

        Los STL de instrumento y marcador se cuelgan de ToolToMarker en sus
        callbacks de carga (son opcionales). El registro (FRW) rellena MarkerToCT.
        """
        markerToCT   = self.logic.getOrCreateTransform("MarkerToCT")
        toolToMarker = self.logic.getOrCreateTransform("ToolToMarker")

        # ToolToMarker cuelga de MarkerToCT: al registrar, el instrumento se alinea
        toolToMarker.SetAndObserveTransformNodeID(markerToCT.GetID())

        # TipCT (punta) cuelga de ToolToMarker: se mueve con el instrumento.
        # Su posicion local es el origen (0,0,0), que es la punta por diseno.
        tip = slicer.mrmlScene.GetFirstNodeByName("TipCT")
        if tip is None:
            tip = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLMarkupsFiducialNode", "TipCT")
            tip.AddControlPoint(0.0, 0.0, 0.0)
            disp = tip.GetDisplayNode()
            if disp is not None:
                disp.SetSelectedColor(1.0, 1.0, 0.0)   # amarillo
                disp.SetPointLabelsVisibility(False)
        tip.SetAndObserveTransformNodeID(toolToMarker.GetID())

    def _setupUIState(self):
        """Lo que no se puede definir en el .ui: escena MRML y contenido del combo."""
        # Los qMRMLNodeComboBox necesitan la escena en tiempo de ejecucion
        for selector in (self.ui.volumeSelector, self.ui.biomodelSelector,
                         self.ui.markerModelSelector, self.ui.instrumentSelector):
            selector.setMRMLScene(slicer.mrmlScene)

        # Modo de operacion: el texto se ve, el dato (currentData) es lo que se pasa al .exe
        self.ui.modeCombo.addItem("Normal",         "normal")
        self.ui.modeCombo.addItem("Ball Profiling", "profiling")
        self.ui.modeCombo.addItem("Calibration",    "calibration")

        # Vista de la ventana del cirujano (espejo). Solo texto; el contenido lo
        # monta _fillSurgeonWindow con widgets, sin tocar el layout global de Slicer.
        self.ui.surgeonViewCombo.addItem("CT slices only")
        self.ui.surgeonViewCombo.addItem("3D only")
        self.ui.surgeonViewCombo.addItem("3D (widescreen) + CT slices")

    def _connectSignals(self):
        self.ui.loadCTBtn.clicked.connect(self._onLoadCT)
        self.ui.loadBiomodelBtn.clicked.connect(self._onLoadBiomodel)
        self.ui.loadMarkerBtn.clicked.connect(self._onLoadMarker)
        self.ui.loadInstrumentBtn.clicked.connect(self._onLoadInstrument)

        self.ui.connectBtn.toggled.connect(self._onConnectToggle)
        self.ui.trackBtn.toggled.connect(self._onTrackToggle)
        self.ui.surgeonDisplayBtn.clicked.connect(self._onToggleSurgeonDisplay)
        self.ui.surgeonViewCombo.currentIndexChanged.connect(self._onSurgeonViewChanged)

        # Registro
        self.ui.captureTipBtn.clicked.connect(self._onCaptureTip)
        self.ui.captureTargetBtn.clicked.connect(self._onCaptureTarget)
        self.ui.computeRegistrationBtn.clicked.connect(self._onComputeRegistration)

        self.ui.modelOpacitySlider.valueChanged.connect(self._onBiomodelOpacity)
        self.ui.markerOpacitySlider.valueChanged.connect(self._onMarkerOpacity)
        self.ui.instrumentOpacitySlider.valueChanged.connect(self._onInstrumentOpacity)
        self.ui.tipOpacitySlider.valueChanged.connect(self._onTipOpacity)
        self.ui.pointsOpacitySlider.valueChanged.connect(self._onPointsOpacity)

        self.ui.showBiomodelCheck.toggled.connect(self._onShowBiomodel)
        self.ui.showMarkerCheck.toggled.connect(self._onShowMarker)
        self.ui.showInstrumentCheck.toggled.connect(self._onShowInstrument)
        self.ui.showTipCheck.toggled.connect(self._onShowTip)
        self.ui.showPointsCheck.toggled.connect(self._onShowPoints)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _onLoadCT(self):
        path = qt.QFileDialog.getOpenFileName(
            None, "Load CT", "", "Volume files (*.nrrd *.nii *.nii.gz *.mha *.mhd *.dcm)"
        )
        if path:
            node = slicer.util.loadVolume(path)
            self.ui.volumeSelector.setCurrentNode(node)   # auto-selección

    def _onLoadBiomodel(self):
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Biomodel", "", "Model files (*.stl *.vtk *.obj *.ply)"
        )
        if path:
            node = slicer.util.loadSegmentation(path)
            self.ui.biomodelSelector.setCurrentNode(node)   # auto-selección

    def _onLoadMarker(self):
        # STL del marcador de referencia. OPCIONAL: solo visualizacion.
        # Cuelga de MarkerToCT (no de ToolToMarker): el marcador es estatico
        # respecto al paciente, asi que tras el registro aparece en su sitio.
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Marker STL", "", "Model files (*.stl *.vtk *.obj)"
        )
        if path:
            node = slicer.util.loadModel(path)
            self.ui.markerModelSelector.setCurrentNode(node)   # auto-selección
            #markerToCT = self.logic.getOrCreateTransform("MarkerToCT")
            #node.SetAndObserveTransformNodeID(markerToCT.GetID())

    def _onLoadInstrument(self):
        # STL del instrumento en coordenadas locales (punta = origen, igual que en C#).
        # OPCIONAL: es solo decorativo. Se cuelga de ToolToMarker, asi que se mueve
        # solo cuando el C# actualiza esa transform (sin recalcular nada en Python).
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Instrument STL", "", "Model files (*.stl *.vtk *.obj)"
        )
        if not path:
            return
        node = slicer.util.loadModel(path)
        self.ui.instrumentSelector.setCurrentNode(node)   # auto-selección

        disp = node.GetDisplayNode()
        if disp is not None:
            disp.SetColor(1.0, 1.0, 0.0)   # amarillo
            disp.SetVisibility2D(False)    # solo en 3D, no en los cortes

        # Colgar el modelo de ToolToMarker (cadena nativa). La cadena ya existe
        # desde setup(); si por lo que fuera no, getOrCreateTransform la crea.
        toolToMarker = self.logic.getOrCreateTransform("ToolToMarker")
        node.SetAndObserveTransformNodeID(toolToMarker.GetID())

    # ── Opacidad ──────────────────────────────────────────────────────────

    def _setOpacity(self, node, value):
        if node is not None and node.GetDisplayNode() is not None:
            node.GetDisplayNode().SetOpacity(value)

    def _onBiomodelOpacity(self, value):
        self._setOpacity(self.ui.biomodelSelector.currentNode(), value)

    def _onMarkerOpacity(self, value):
        self._setOpacity(self.ui.markerModelSelector.currentNode(), value)

    def _onInstrumentOpacity(self, value):
        self._setOpacity(self.ui.instrumentSelector.currentNode(), value)

    def _onTipOpacity(self, value):
        self._setOpacity(slicer.mrmlScene.GetFirstNodeByName("TipCT"), value)

    def _onPointsOpacity(self, value):
        for name in ("RASPoints", "ReferencePoints"):
            self._setOpacity(slicer.mrmlScene.GetFirstNodeByName(name), value)

    # ── Visibilidad (checkboxes) ──────────────────────────────────────────

    def _setNodeVisible(self, node, visible):
        if node is not None and node.GetDisplayNode() is not None:
            node.GetDisplayNode().SetVisibility(visible)

    def _onShowBiomodel(self, checked):
        self._setNodeVisible(self.ui.biomodelSelector.currentNode(), checked)

    def _onShowMarker(self, checked):
        self._setNodeVisible(self.ui.markerModelSelector.currentNode(), checked)

    def _onShowInstrument(self, checked):
        self._setNodeVisible(self.ui.instrumentSelector.currentNode(), checked)

    def _onShowTip(self, checked):
        self._setNodeVisible(slicer.mrmlScene.GetFirstNodeByName("TipCT"), checked)

    def _onShowPoints(self, checked):
        # Oculta/muestra las dos listas de fiduciales del registro a la vez
        # (util para 'limpiar' la escena una vez conseguido un buen registro).
        for name in ("RASPoints", "ReferencePoints"):
            self._setNodeVisible(slicer.mrmlScene.GetFirstNodeByName(name), checked)

    def _onToggleSurgeonDisplay(self):
        """Abre o cierra la ventana secundaria del cirujano."""
        if self._secondaryWindow is not None:
            self._secondaryWindow.close()
            self._secondaryWindow = None
            self.ui.surgeonDisplayBtn.setText("🖥  Open surgeon display")
            return

        self._secondaryWindow = self._buildSurgeonWindow()
        self.ui.surgeonDisplayBtn.setText("✖  Close surgeon display")

    def _onSurgeonViewChanged(self, index):
        """Si la ventana esta abierta, reconstruye su contenido con la vista elegida."""
        if self._secondaryWindow is None:
            return
        self._fillSurgeonWindow(self._secondaryWindow)

    def _buildSurgeonWindow(self):
        """Crea la QMainWindow del cirujano y la rellena segun el selector de vista.
        Es un ESPEJO: reutiliza los slice/3D nodes de la escena principal, sin tocar
        el layout global de Slicer. Si hay segundo monitor, se lanza alli."""
        win = qt.QMainWindow()
        win.setWindowTitle("Surgical Navigator — Surgeon display")
        self._fillSurgeonWindow(win)

        screens = slicer.app.screens() if hasattr(slicer.app, "screens") else []
        if len(screens) > 1:
            geo = screens[1].geometry
            win.move(geo.x(), geo.y())
            win.resize(geo.width(), geo.height())
            win.show()
        else:
            win.resize(1200, 500)
            win.show()
        return win

    def _fillSurgeonWindow(self, win):
        """Monta el widget central de la ventana del cirujano segun la vista elegida:
        'CT slices only' (3 cortes), '3D only', o 'Conventional widescreen' (3D + cortes).
        Reconstruye desde cero cada vez que se llama (al abrir o al cambiar el selector)."""
        view = self.ui.surgeonViewCombo.currentText

        central = qt.QWidget()
        hbox = qt.QHBoxLayout(central)
        hbox.setContentsMargins(0, 0, 0, 0)
        hbox.setSpacing(2)

        def sliceWidget(color):
            node = slicer.mrmlScene.GetFirstNodeByName(color)
            sw = slicer.qMRMLSliceWidget()
            sw.setMRMLScene(slicer.mrmlScene)
            sw.setMRMLSliceNode(node)
            return sw

        def threeDWidget():
            tw = slicer.qMRMLThreeDWidget()
            tw.setMRMLScene(slicer.mrmlScene)
            viewNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLViewNode")
            if viewNode is not None:
                tw.setMRMLViewNode(viewNode)
            return tw

        if view == "3D only":
            hbox.addWidget(threeDWidget())
        elif view == "3D (widescreen) + CT slices":
            vbox = qt.QVBoxLayout()
            vbox.setContentsMargins(0, 0, 0, 0)
            vbox.setSpacing(2)
            vbox.addWidget(threeDWidget(), 3)      # 3D grande arriba
            slicesRow = qt.QHBoxLayout()
            slicesRow.setSpacing(2)
            for color in ("Red", "Yellow", "Green"):
                slicesRow.addWidget(sliceWidget(color))
            vbox.addLayout(slicesRow, 2)           # cortes en fila debajo
            hbox.addLayout(vbox)
        else:   # "CT slices only" (o cualquier otro): los 3 cortes
            for color in ("Red", "Yellow", "Green"):
                hbox.addWidget(sliceWidget(color))

        win.setCentralWidget(central)

    # ── Registro ──────────────────────────────────────────────────────────

    def _captureStablePosition(self, getter, label):
        """Acumula muestras durante unos 5 s y rechaza posiciones inestables."""
        samples = []
        timer = qt.QElapsedTimer()
        timer.start()

        durationMs = 5000
        sampleIntervalMs = 50
        nextSampleMs = 0

        while timer.elapsed() < durationMs:
            elapsed = timer.elapsed()

            if elapsed >= nextSampleMs:
                pos = getter()
                if pos is not None:
                    samples.append(np.asarray(pos, dtype=float))
                nextSampleMs += sampleIntervalMs

            slicer.app.processEvents()

            import time
            time.sleep(0.005)

        if len(samples) < 3:
            slicer.util.warningDisplay(
                f"{label}: not enough tracking samples during the 5 s capture.")
            return None

        data = np.vstack(samples)
        median = np.median(data, axis=0)

        deviations = data - median
        radialRms = float(
            np.sqrt(np.mean(np.sum(deviations ** 2, axis=1)))
        )

        if radialRms > 6.0:
            slicer.util.warningDisplay(
                f"{label}: unstable capture "
                f"(RMS dispersion {radialRms:.2f} mm > 6.00 mm). "
                "Point rejected. Keep the tip still and try again.")
            return None

        slicer.util.infoDisplay(
            f"{label}: point captured "
            f"({len(samples)} samples, dispersion {radialRms:.2f} mm)."
        )

        return median.tolist()

    def _onCaptureTip(self):
        """Captura la posicion actual de la punta (origen de ToolToMarker, en coords
        del marcador) y la anade a ReferencePoints. Equivale al 'Place From' del FRW.
        La tabla de la derecha refleja el nuevo punto automaticamente."""
        ref = slicer.mrmlScene.GetFirstNodeByName("ReferencePoints")
        if ref is None:
            slicer.util.warningDisplay(
                "Reference point list missing. Reload the module.")
            return

        pos = self._captureStablePosition(
            self.logic.getToolToMarkerOrigin,
            "Tip capture"
        )

        if pos is None:
            return

        ref.AddControlPoint(
            float(pos[0]),
            float(pos[1]),
            float(pos[2])
        )

    def _onCaptureTarget(self):
        """Captura la posicion de la punta EN ESPACIO CT (getTipWorld, que lee TipCT
        ya transformado por la cadena) y la anade a la lista 'Targets'. Para medir TRE:
        estos puntos se comparan con las dianas marcadas en el STL. NO entran en el FRW."""
        tgt = self.logic.getOrCreatePointList(
            "Targets", color=(0.1, 0.9, 0.2)
        )

        pos = self._captureStablePosition(
            self.logic.getTipWorld,
            "Target capture"
        )

        if pos is None:
            return

        tgt.AddControlPoint(
            float(pos[0]),
            float(pos[1]),
            float(pos[2])
        )

    def _onComputeRegistration(self):
        """Invoca el Fiducial Registration Wizard de SlicerIGT para calcular
        MarkerToCT a partir de ReferencePoints (From) y RASPoints (To)."""
        ref = slicer.mrmlScene.GetFirstNodeByName("ReferencePoints")
        ras = slicer.mrmlScene.GetFirstNodeByName("RASPoints")
        if ref is None or ras is None:
            self.ui.regResultLabel.setText("⚠ Point lists missing. Reload the module.")
            return
        nRef = ref.GetNumberOfControlPoints()
        nRas = ras.GetNumberOfControlPoints()
        if nRef < 3 or nRas < 3:
            self.ui.regResultLabel.setText(
                f"⚠ Need ≥3 matched points (have {nRef} tip, {nRas} CT).")
            return
        if nRef != nRas:
            self.ui.regResultLabel.setText(
                f"⚠ Counts differ: {nRef} tip vs {nRas} CT. They must match 1-to-1.")
            return

        markerToCT = self.logic.getOrCreateTransform("MarkerToCT")
        try:
            rms = self.logic.runFiducialRegistration(ref, ras, markerToCT)
        except Exception as e:
            self.ui.regResultLabel.setText(f"⚠ Registration failed: {e}")
            return

        if rms is None:
            self.ui.regResultLabel.setText(
                "Registration computed (RMS not reported). Check alignment in 3D.")
        else:
            color = "#27ae60" if rms < 3.0 else "#e67e22" if rms < 6.0 else "#e74c3c"
            self.ui.regResultLabel.setText(f"RMS error: {rms:.2f} mm")
            self.ui.regResultLabel.setStyleSheet(
                f"font-size: 12px; font-weight: bold; color: {color};")

    def cleanup(self):
        """Se llama cuando Slicer descarga el módulo. Cerramos la ventana
        secundaria si estuviera abierta para evitar widgets huérfanos."""
        if getattr(self, "_secondaryWindow", None) is not None:
            self._secondaryWindow.close()
            self._secondaryWindow = None

    def _onConnectToggle(self, checked):
        if checked:
            # Limpiar connector anterior si quedó de un intento previo
            old = slicer.mrmlScene.GetFirstNodeByName("CTNavigatorConnector")
            if old is not None:
                old.Stop()
                slicer.mrmlScene.RemoveNode(old)
            
            # 1. Lanzar el .exe con el modo elegido
            arg = self.ui.modeCombo.currentData
            self._proc = qt.QProcess()
            self._proc.started.connect(lambda: self._setConnStatus("process started", "green"))
            self._proc.errorOccurred.connect(
                lambda e: self._setConnStatus(f"error launching exe: {e}", "red"))
            self._proc.start("cmd.exe", [
                "/c", "start", "",
                "C:/TFM/CTNavigator-clone/KinectTracker/Kinect-PLUS/bin/Debug/Kinect-PLUS.exe",
                arg
            ])

            # 2. Crear y arrancar el connector (cliente hacia el server del C#)
            self._connector = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLIGTLConnectorNode", "CTNavigatorConnector")
            self._connector.SetTypeClient("localhost", 18944)

            # 3. Suscribirse a sus eventos ANTES de arrancar
            self._connector.AddObserver(
                slicer.vtkMRMLIGTLConnectorNode.ConnectedEvent,
                lambda c, e: self._setConnStatus("connected to tracker", "green"))
            self._connector.AddObserver(
                slicer.vtkMRMLIGTLConnectorNode.DisconnectedEvent,
                lambda c, e: self._setConnStatus("waiting for tracker...", "orange"))

            # 4. Ahora sí, arrancar
            self._connector.Start()
            self._setConnStatus("waiting for tracker...", "orange")

            self.ui.connectBtn.setText("🔌  Disconnect")
        else:
            # Parar connector y proceso
            if getattr(self, "_connector", None):
                self._connector.Stop()
                slicer.mrmlScene.RemoveNode(self._connector)
                self._connector = None
            if getattr(self, "_proc", None):
                self._proc.kill()
                self._proc = None
            self.ui.connectBtn.setText("🔌  Connect")
            self._setConnStatus("disconnected", "gray")

    def _subscribeToTool(self, toolNode):
        # Suscribir el observer al node del instrumento
        self.ui.trackBtn.setText("■  Stop tracking")
        self._toolObserverTag = toolNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent,
            self._onToolMoved
        )

    @vtk.calldata_type(vtk.VTK_OBJECT)
    def _onNodeAdded(self, caller, event, calldata):
        # Cuando el C# empieza a enviar, el node ToolToMarker aparece en la escena.
        # Al aparecer, lo colgamos de MarkerToCT (por si se creo despues de setup)
        # y enganchamos el observer del label.
        node = calldata
        if node is not None and node.GetName() == "ToolToMarker":
            markerToCT = self.logic.getOrCreateTransform("MarkerToCT")
            node.SetAndObserveTransformNodeID(markerToCT.GetID())
            self._subscribeToTool(node)
            # Ya está enganchado; dejamos de vigilar la escena
            if hasattr(self, "_sceneObserver"):
                slicer.mrmlScene.RemoveObserver(self._sceneObserver)
                del self._sceneObserver

    def _onTrackToggle(self, checked):
        if checked:
            # La cadena nativa mueve el instrumento sola; el observer es solo para
            # refrescar el label de posicion. Se engancha a ToolToMarker, que es la
            # transform que el C# actualiza y de la que cuelga la punta.
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToMarker")
            if toolNode is not None:
                self._subscribeToTool(toolNode)
            else:
                # Aun no ha llegado; nos enganchamos cuando se vea
                self._sceneObserver = slicer.mrmlScene.AddObserver(
                    slicer.mrmlScene.NodeAddedEvent, self._onNodeAdded)
                self.ui.trackBtn.setText("■  Stop tracking")
        else:
            self.ui.trackBtn.setText("▶  Start tracking")
            # Quitar el observer
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToMarker")
            if toolNode is not None and hasattr(self, "_toolObserverTag"):
                toolNode.RemoveObserver(self._toolObserverTag)
            # Si aún estábamos esperando a que apareciera el node, dejar de vigilar la escena
            if hasattr(self, "_sceneObserver"):
                slicer.mrmlScene.RemoveObserver(self._sceneObserver)
                del self._sceneObserver

    def _setConnStatus(self, text, color):
        self.ui.connStatusLabel.setText(f"● {text}")
        self.ui.connStatusLabel.setStyleSheet(f"color: {color}; font-size: 11px;")


    def _onToolMoved(self, caller, event):
        # Con la cadena nativa (MarkerToCT -> ToolToMarker -> TipCT), Slicer ya
        # mueve el instrumento y la punta solos. Aqui solo saltamos los cortes
        # del CT a la posicion actual de la punta.
        try:
            pos = self.logic.getTipWorld()
            if pos is None:
                return
            self.logic.jumpToRAS(pos)
        except Exception:
            pass
# ─────────────────────────────────────────────────────────────────────────────
# Logic
# ─────────────────────────────────────────────────────────────────────────────

class CTNavigatorLogic(ScriptedLoadableModuleLogic):
    """
    Cadena de transformadas:

        T_CT_pen = inv(T_Star2CT) · inv(T_Tracker2Star) · T_Tracker2Pen

    T_Star2CT:
        Se calcula como la traslación al centroide de las 3 bolas IR
        leído desde los markups con GetNthControlPointPositionWorld().
        Como los markups comparten transform con el STL del marcador,
        sus posiciones mundiales ya reflejan dónde está el marcador en el CT.

    T_Tracker2Star / T_Tracker2Pen:
        Traslaciones puras con las XYZ simuladas (sin rotación).
        Con la cámara serán matrices 4×4 completas desde PLUS.
        La cadena no cambia.
    """

    def jumpToRAS(self, ras):
        """Mueve las tres vistas del CT al punto RAS dado."""
        slicer.modules.markups.logic().JumpSlicesToLocation(
            float(ras[0]), float(ras[1]), float(ras[2]), True
        )

    def getTipWorld(self):
        """Posicion mundial (RAS) de la punta: el control point 0 de TipCT,
        que cuelga de la cadena ToolToMarker -> MarkerToCT. Devuelve None si
        aun no existe."""
        tip = slicer.mrmlScene.GetFirstNodeByName("TipCT")
        if tip is None or tip.GetNumberOfControlPoints() == 0:
            return None
        p = [0.0, 0.0, 0.0]
        tip.GetNthControlPointPositionWorld(0, p)
        return p

    def getOrCreateTransform(self, nodeName):
        """Devuelve el transform node con ese nombre, creándolo si no existe."""
        node = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLLinearTransformNode", nodeName)
        return node

    def getOrCreatePointList(self, nodeName, color=None):
        """Devuelve (o crea) una lista de fiduciales por nombre. Se deja en la
        RAIZ, sin transform padre: el FRW exige que los puntos esten en el
        sistema que se registra, no re-transformados por una cadena."""
        node = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLMarkupsFiducialNode", nodeName)
            disp = node.GetDisplayNode()
            if disp is not None and color is not None:
                disp.SetSelectedColor(*color)
        return node

    def getToolToMarkerOrigin(self):
        """Posicion del origen de ToolToMarker = la punta en coords del marcador.
        Es lo que el FRW capturaria con 'Place From'. Devuelve None si no existe."""
        node = slicer.mrmlScene.GetFirstNodeByName("ToolToMarker")
        if node is None:
            return None
        m = vtk.vtkMatrix4x4()
        node.GetMatrixTransformToParent(m)
        return [m.GetElement(0, 3), m.GetElement(1, 3), m.GetElement(2, 3)]

    def runFiducialRegistration(self, fromList, toList, outputTransform):
        """Calcula el registro rigido From->To usando el Fiducial Registration
        Wizard de SlicerIGT y lo escribe en outputTransform. Devuelve el RMS (mm)
        o None si no se pudo leer.

        Se usa el nodo de parametros del FRW (vtkMRMLFiducialRegistrationWizardNode)
        y su logic. Los nombres de metodo variaron entre versiones de SlicerIGT,
        asi que se prueban las variantes conocidas de forma defensiva.
        """
        # Nodo de parametros del FRW
        frwNode = slicer.mrmlScene.AddNewNodeByClass(
            "vtkMRMLFiducialRegistrationWizardNode")
        try:
            frwNode.SetAndObserveFromFiducialListNodeId(fromList.GetID())
            frwNode.SetAndObserveToFiducialListNodeId(toList.GetID())
            frwNode.SetOutputTransformNodeId(outputTransform.GetID())

            # Modo rigido (probar las variantes de API conocidas)
            if hasattr(frwNode, "SetRegistrationModeToRigid"):
                frwNode.SetRegistrationModeToRigid()
            elif hasattr(frwNode, "SetRegistrationMode"):
                frwNode.SetRegistrationMode("Rigid")

            # Emparejamiento manual (los puntos ya estan en orden 1-a-1)
            if hasattr(frwNode, "SetPointMatchingToManual"):
                frwNode.SetPointMatchingToManual()
            #elif hasattr(frwNode, "SetPointMatching"):
              #  frwNode.SetPointMatching("Manual")

            frwLogic = slicer.modules.fiducialregistrationwizard.logic()

            # El calculo se dispara de una de estas formas segun la version
            if hasattr(frwLogic, "UpdateCalibration"):
                frwLogic.UpdateCalibration(frwNode)
            elif hasattr(frwLogic, "UpdateCalibrationInternal"):
                frwLogic.UpdateCalibrationInternal(frwNode)
            elif hasattr(frwLogic, "CalculateTransform"):
                frwLogic.CalculateTransform(frwNode)
            # Si ninguno existe, algunas versiones calculan solo al observar inputs;
            # forzamos un Modified por si acaso
            frwNode.Modified()

            # Leer el RMS del resultado (nombre del getter varia)
            rms = None
            for getter in ("GetCalibrationError", "GetError", "GetRMSError"):
                if hasattr(frwNode, getter):
                    try:
                        rms = float(getattr(frwNode, getter)())
                        break
                    except Exception:
                        pass
            return rms
        finally:
            # El nodo de parametros es temporal; lo quitamos para no ensuciar la escena.
            # El resultado ya quedo escrito en outputTransform.
            slicer.mrmlScene.RemoveNode(frwNode)

    def readTransformMatrix(self, nodeName):
        """
        Lee la matriz 4x4 de un transform node por nombre.
        Devuelve np.array (4,4), o None si el node no existe.
        (Se conserva por utilidad general; la cadena nativa no la necesita.)
        """
        node = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if node is None:
            return None

        vtkMat = vtk.vtkMatrix4x4()
        node.GetMatrixTransformToParent(vtkMat)

        M = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                M[i, j] = vtkMat.GetElement(i, j)
        return M