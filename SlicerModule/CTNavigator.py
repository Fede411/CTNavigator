"""
# References:
# [1] Haxthausen et al., "UltrARsound: in situ visualization of live
#     ultrasound images using HoloLens 2", IJCARS 17, 2081-2091 (2022)
#     https://doi.org/10.1007/s11548-022-02695-z
# [2] BSEL-UC3M, "OpenARHealth", GitHub (2022)
#     https://github.com/BSEL-UC3M/OpenARHealth
# [3] Sevilla Garcia et al., "3D Slicer Module Implementation in Python",
#     BSEL-UC3M / IGT Workshop (2025)

CTNavigator - 3D Slicer Module (v2)
=====================================
Modulo de navegacion quirurgica para CT. Calcula la posicion del
instrumento en el CT a partir de la posicion del marcador estrella
IR y del instrumento en el espacio del tracker.

Estructura del modulo:
  SETUP
    LOAD
      Load files  -> carga CT, biomodelo, marcador STL y crea StarBalls
      Scene nodes -> selectores de nodos ya cargados en la escena
    CONNECTION    -> PLUS/OpenIGTLink (pendiente)
  TOGGLES
    Opacidad del biomodelo, marcador y CT

Cadena de transformadas:
  T_Star2CT  = centroide de los 3 markups StarBalls en espacio CT
  T_CT_pen   = inv(T_Star2CT) * inv(T_Tracker2Star) * T_Tracker2Pen
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
        self.parent.title        = "CT Navigator"
        self.parent.categories   = ["Navigation"]
        self.parent.dependencies = []
        self.parent.contributors = [""]
        self.parent.helpText = (
            "Calcula la posición del lápiz en el CT a partir de:\n"
            "· Los 3 markups de las bolas IR del marcador (T_Star2CT)\n"
            "· La transformada del marcador en el tracker (MarkerToTracker)\n"
            "· La transformada del lápiz en el tracker (ToolToTracker)"
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
        self._connectSignals()

    def _setupUIState(self):
        """Lo que no se puede definir en el .ui: escena MRML y contenido del combo."""
        # Los qMRMLNodeComboBox necesitan la escena en tiempo de ejecucion
        for selector in (self.ui.volumeSelector, self.ui.biomodelSelector,
                         self.ui.markerModelSelector, self.ui.instrumentSelector,
                         self.ui.ballsSelector):
            selector.setMRMLScene(slicer.mrmlScene)

        # Modo de operacion: el texto se ve, el dato (currentData) es lo que se pasa al .exe
        self.ui.modeCombo.addItem("Normal",         "normal")
        self.ui.modeCombo.addItem("Ball Profiling", "profiling")
        self.ui.modeCombo.addItem("Calibration",    "calibration")

    def _connectSignals(self):
        self.ui.loadCTBtn.clicked.connect(self._onLoadCT)
        self.ui.loadBiomodelBtn.clicked.connect(self._onLoadBiomodel)
        self.ui.loadMarkerBtn.clicked.connect(self._onLoadMarker)
        self.ui.loadInstrumentBtn.clicked.connect(self._onLoadInstrument)
        self.ui.createStarBallsBtn.clicked.connect(self._onCreateStarBalls)
        self.ui.readBtn.clicked.connect(self._readBalls)

        self.ui.connectBtn.toggled.connect(self._onConnectToggle)
        self.ui.trackBtn.toggled.connect(self._onTrackToggle)
        self.ui.surgeonDisplayBtn.clicked.connect(self._onToggleSurgeonDisplay)

        self.ui.modelOpacitySlider.valueChanged.connect(self._onBiomodelOpacity)
        self.ui.markerOpacitySlider.valueChanged.connect(self._onMarkerOpacity)
        self.ui.ctOpacitySlider.valueChanged.connect(self._onCTOpacity)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _readBalls(self):
        """Lee el centroide actual de las bolas y lo muestra."""
        balls = self.ui.ballsSelector.currentNode()
        if balls is None:
            self.ui.centroidLabel.setText("(no markup selected)")
            return
        if balls.GetNumberOfControlPoints() < 3:
            self.ui.centroidLabel.setText("⚠ You need 3 control points")
            return
        try:
            centroid = self.logic.getCentroidInCT(balls)
            self.ui.centroidLabel.setText(
                f"R={centroid[0]:+.1f}  A={centroid[1]:+.1f}  S={centroid[2]:+.1f} mm"
            )
        except Exception as e:
            self.ui.centroidLabel.setText(f"Error: {e}")

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
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Marker STL", "", "Model files (*.stl *.vtk *.obj)"
        )
        if path:
            node = slicer.util.loadModel(path)
            self.ui.markerModelSelector.setCurrentNode(node)   # auto-selección

    def _onLoadInstrument(self):
        # STL del instrumento en coordenadas locales (punta = origen, igual que en C#).
        # Se pinta amarillo, se ve solo en 3D y se cuelga del transform ToolToCT,
        # que el tracking actualiza en cada frame para moverlo en tiempo real.
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

        # Colgar el modelo del transform ToolToCT (lo crea si no existe todavía)
        toolToCT = self.logic.getOrCreateTransform("ToolToCT")
        node.SetAndObserveTransformNodeID(toolToCT.GetID())

    def _onCreateStarBalls(self):
        # Eliminar StarBalls previo si existe
        existing = slicer.util.getNodes("StarBalls")
        for node in existing.values():
            slicer.mrmlScene.RemoveNode(node)

        node = slicer.mrmlScene.AddNewNodeByClass(
            "vtkMRMLMarkupsFiducialNode", "StarBalls"
        )
        node.AddControlPoint(  1.139,  55.872, 15.805)
        node.AddControlPoint( 34.993,  -5.779, 15.913)
        node.AddControlPoint(-35.161,  -5.600, 15.928)
        slicer.util.showStatusMessage("StarBalls create with 3 points.", 3000)

    def _onBiomodelOpacity(self, value):
        node = self.ui.biomodelSelector.currentNode()
        if node is None:
            return
        node.GetDisplayNode().SetOpacity(value)

    def _onMarkerOpacity(self, value):
        node = self.ui.markerModelSelector.currentNode()
        if node is None:
            return
        node.GetDisplayNode().SetOpacity(value)

    def _onCTOpacity(self, value):
        vol = self.ui.volumeSelector.currentNode()
        if vol is None:
            return
        slicer.util.setSliceViewerLayers(background=vol, backgroundOpacity=value)

    def _onToggleSurgeonDisplay(self):
        """
        Toggle de la ventana secundaria para el cirujano.
        Si está cerrada, la abre (y la envía al 2º monitor si existe).
        Si está abierta, la cierra.
        """
        if self._secondaryWindow is not None:
            # Ya existe una ventana abierta → cerrar
            self._secondaryWindow.close()
            self._secondaryWindow = None
            self.ui.surgeonDisplayBtn.setText("🖥  Open surgeon display")
            return

        # Crear ventana nueva
        self._secondaryWindow = self._buildSurgeonWindow()
        self.ui.surgeonDisplayBtn.setText("✖  Close surgeon display")

    def _buildSurgeonWindow(self):
        """
        Crea una QMainWindow con tres qMRMLSliceWidget (Red/Yellow/Green)
        que comparten los slice nodes de las vistas principales, de modo
        que cualquier cambio en el layout del operador se refleja aquí.

        Si hay un segundo monitor, la ventana se lanza allí en fullscreen.
        """
        win = qt.QMainWindow()
        win.setWindowTitle("CT Navigator — Surgeon display")

        # Widget central con las 3 vistas en horizontal
        central = qt.QWidget()
        hbox    = qt.QHBoxLayout(central)
        hbox.setContentsMargins(0, 0, 0, 0)
        hbox.setSpacing(2)

        layoutManager = slicer.app.layoutManager()

        for color in ("Red", "Yellow", "Green"):
            # Obtenemos el SliceNode que ya existe en la escena principal
            sliceNode = slicer.mrmlScene.GetFirstNodeByName(color)

            # Creamos un nuevo widget que apunta a ese mismo nodo
            sw = slicer.qMRMLSliceWidget()
            sw.setMRMLScene(slicer.mrmlScene)
            sw.setMRMLSliceNode(sliceNode)

            hbox.addWidget(sw)

        win.setCentralWidget(central)

        # Decidir dónde mostrar la ventana
        win.setCentralWidget(central)
        win.resize(1200, 400)   
        win.show()
        return win

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
            self._proc.started.connect(lambda: self._setConnStatus("proceso arrancado", "green"))
            self._proc.errorOccurred.connect(
                lambda e: self._setConnStatus(f"error al arrancar exe: {e}", "red"))
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
                lambda c, e: self._setConnStatus("conectado al tracker", "green"))
            self._connector.AddObserver(
                slicer.vtkMRMLIGTLConnectorNode.DisconnectedEvent,
                lambda c, e: self._setConnStatus("esperando al tracker...", "orange"))

            # 4. Ahora sí, arrancar
            self._connector.Start()
            self._setConnStatus("esperando al tracker...", "orange")

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

    def _subscribeToTool(self, toolNode):
        # Suscribir el observer al node del instrumento
        self.ui.trackBtn.setText("■  Stop tracking")
        self._toolObserverTag = toolNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent,
            self._onToolMoved
        )

    @vtk.calldata_type(vtk.VTK_OBJECT)
    def _onNodeAdded(self, caller, event, calldata):
        # Cuando el C# empieza a enviar, el node ToolToTracker aparece en la escena
        node = calldata
        if node is not None and node.GetName() == "ToolToTracker":
            self._subscribeToTool(node)
            # Ya está enganchado; dejamos de vigilar la escena
            if hasattr(self, "_sceneObserver"):
                slicer.mrmlScene.RemoveObserver(self._sceneObserver)
                del self._sceneObserver

    def _onTrackToggle(self, checked):
        if checked:
            # Verificar StarBalls
            balls = self.ui.ballsSelector.currentNode()
            if balls is None or balls.GetNumberOfControlPoints() < 3:
                slicer.util.warningDisplay("Select StarBalls (3 points) before tracking.")
                self.ui.trackBtn.setChecked(False)
                return

            # Verificar que el node del instrumento ya llega
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToTracker")
            if toolNode is not None:
                self._subscribeToTool(toolNode)
            else:
                # Aún no ha llegado; nos enganchamos cuando se vea
                self._sceneObserver = slicer.mrmlScene.AddObserver(
                    slicer.mrmlScene.NodeAddedEvent, self._onNodeAdded)
                self.ui.trackBtn.setText("■  Stop tracking")
        else:
            self.ui.trackBtn.setText("▶  Start tracking")
            # Quitar el observer
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToTracker")
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
        balls = self.ui.ballsSelector.currentNode()
        if balls is None:
            return

        T_star2tracker = self.logic._readTransform("MarkerToTracker")
        T_pen2tracker  = self.logic._readTransform("ToolToTracker")

        if T_star2tracker is None or T_pen2tracker is None:
            return

        try:
            T_pen2ct = self.logic.computePenInCT(balls, T_star2tracker, T_pen2tracker)
            pos = T_pen2ct[:3, 3]
            self.ui.penCtLabel.setText(
                f"R={pos[0]:+.1f}  A={pos[1]:+.1f}  S={pos[2]:+.1f}"
            )
            # Mover el STL del instrumento en 3D (pose completa, con rotación)
            self.logic.updateToolTransform(T_pen2ct)
            # Pintar la punta en los cortes y saltar el corte a ella
            self.logic.updateTipMarkup(pos)
            self.logic.jumpToRAS(pos)
        except Exception as e:
            self.ui.errorLabel.setText(f"⚠ Error: {e}")
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

    def computePenInCT(self, ballsNode, T_star2tracker, T_pen2tracker):
        T_star2ct = self.computeStarToCT(ballsNode)
        T_pen2ct = (
            T_star2ct
            @ np.linalg.inv(T_star2tracker)
            @ T_pen2tracker
        )
        return T_pen2ct

    def getCentroidInCT(self, ballsNode):
        """
        Lee las posiciones mundiales de los 3 markups (ya con la transform
        aplicada porque comparten transform con el STL) y devuelve
        el centroide en espacio CT (RAS, mm).
        """
        pts = []
        for i in range(3):
            p = [0.0, 0.0, 0.0]
            ballsNode.GetNthControlPointPositionWorld(i, p)
            pts.append(p)
        return np.mean(pts, axis=0)

    def jumpToRAS(self, ras):
        """Mueve las tres vistas del CT al punto RAS dado."""
        slicer.modules.markups.logic().JumpSlicesToLocation(
            float(ras[0]), float(ras[1]), float(ras[2]), True
        )

    def computeStarToCT(self, ballsNode):
        """
        Calcula T_Star2CT (4x4) alineando las 3 bolas en coordenadas
        locales del marcador con las 3 bolas markup en el CT (Horn/SVD).
        """
        # Bolas en coordenadas locales del marcador (mismas que CreateMarker en C#)
        local = np.array([
            [ 12.319,   3.206,   0.0],
            [-13.660,   4.397,  15.0],
            [  1.340,  -7.603, -15.0],
        ])

        # Bolas en el CT (markups StarBalls, en orden A, B, C)
        ct = []
        for i in range(3):
            p = [0.0, 0.0, 0.0]
            ballsNode.GetNthControlPointPositionWorld(i, p)
            ct.append(p)
        ct = np.array(ct)

        R, t = self._horn(local, ct)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def _horn(self, src, dst):
        """
        Horn/SVD: encuentra R, t tal que dst ≈ R·src + t.
        src, dst: arrays (N,3). Devuelve R (3x3), t (3,).
        """
        # 1. Centroides
        centroid_src = np.mean(src, axis=0) 
        centroid_dst = np.mean(dst, axis=0)

        # 2. Centrar
        src_c = src - centroid_src
        dst_c = dst - centroid_dst

        # 3. Matriz H = Σ src_c[i] outer dst_c[i]  →  en numpy: src_c.T @ dst_c
        H = src_c.T @ dst_c

        # 4. SVD
        U, S, Vt = np.linalg.svd(H)

        # 5. R = V · U^T   (ojo: numpy da Vt = V^T, así que V = Vt.T)
        R = Vt.T @ U.T

        # 6. Corrección de reflexión si det(R) < 0
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # 7. t = centroid_dst - R · centroid_src
        t = centroid_dst - R @ centroid_src

        return R, t

    def getOrCreateTransform(self, nodeName):
        """Devuelve el transform node con ese nombre, creándolo si no existe."""
        node = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLLinearTransformNode", nodeName)
        return node

    def updateToolTransform(self, T_pen2ct):
        """
        Escribe la pose 4x4 del instrumento (en CT) en el transform ToolToCT.
        El STL del instrumento cuelga de este node, así que se mueve con él.
        """
        node = self.getOrCreateTransform("ToolToCT")
        vtkMat = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtkMat.SetElement(i, j, float(T_pen2ct[i, j]))
        node.SetMatrixTransformToParent(vtkMat)

    def updateTipMarkup(self, pos):
        """
        Pinta la punta como un markup de 1 punto (visible en 3D y en los cortes).
        Lo crea la primera vez y luego solo mueve el control point.
        """
        node = slicer.mrmlScene.GetFirstNodeByName("TipCT")
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLMarkupsFiducialNode", "TipCT")
            node.AddControlPoint(float(pos[0]), float(pos[1]), float(pos[2]))
            disp = node.GetDisplayNode()
            if disp is not None:
                disp.SetSelectedColor(1.0, 1.0, 0.0)   # amarillo, a juego con el STL
                disp.SetPointLabelsVisibility(False)
        else:
            node.SetNthControlPointPositionWorld(
                0, float(pos[0]), float(pos[1]), float(pos[2]))

    def _readTransform(self, nodeName):
        """
        Lee la matriz 4x4 de un transform node por nombre.
        Devuelve np.array (4,4), o None si el node no existe.
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