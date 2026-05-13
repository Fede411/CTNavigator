"""
# References:
# [1] Haxthausen et al., "UltrARsound: in situ visualization of live
#     ultrasound images using HoloLens 2", IJCARS 17, 2081-2091 (2022)
#     https://doi.org/10.1007/s11548-022-02695-z
# [2] BSEL-UC3M, "OpenARHealth", GitHub (2022)
#     https://github.com/BSEL-UC3M/OpenARHealth
# [3] Sevilla Garcia et al., "3D Slicer Module Implementation in Python",
#     BSEL-UC3M / IGT Workshop (2025)

CTNavigator - 3D Slicer Module (v1)
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
  COORDINATE VISUALIZER
    Coordenadas XYZ del marcador en espacio tracker (simuladas)
    Coordenadas XYZ del instrumento en espacio tracker (simuladas)
    Boton de calculo -> salta laminas del CT a la posicion calculada
  MOVEMENT SIMULATION
    Trayectorias automaticas para validacion sin camara
  TOGGLES
    Opacidad del biomodelo, marcador y CT

Cadena de transformadas:
  T_Star2CT  = centroide de los 3 markups StarBalls en espacio CT
  T_CT_pen   = inv(T_Star2CT) * inv(T_Tracker2Star) * T_Tracker2Pen

Geometria del marcador estrella al importarlo (coordenadas locales, mm):
  Bola 1:  ( 1.139,  55.872, 15.805)
  Bola 2:  (34.993,  -5.779, 15.913)
  Bola 3:  (-35.161, -5.600, 15.928)
  Enrosque: 10.903 mm altura, bola IR 13 mm diametro

Cuando llegue la camara:
  Sustituir los spinboxes por transforms de PLUS/OpenIGTLink.
  La cadena de transformadas no cambia.
"""

import math
import random
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
            "· Coordenadas simuladas del marcador en el tracker\n"
            "· Coordenadas simuladas del lápiz en el tracker"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Widget
# ─────────────────────────────────────────────────────────────────────────────

class CTNavigatorWidget(ScriptedLoadableModuleWidget):

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        self.logic  = CTNavigatorLogic()
        self._timer = qt.QTimer()
        self._timer.timeout.connect(self._tick)
        self._buildUI()

    def _buildUI(self):
        layout = self.layout

        #Carga de archivos
        #SETUP
        setupBox = ctk.ctkCollapsibleButton()
        setupBox.text = "SETUP"
        layout.addWidget(setupBox)
        setupLayout = qt.QVBoxLayout(setupBox)

        #LOAD
        loadBox = ctk.ctkCollapsibleButton()
        loadBox.text = "LOAD"
        setupLayout.addWidget(loadBox)
        loadLayout = qt.QVBoxLayout(loadBox)

        #Load files
        loadFilesBox = ctk.ctkCollapsibleButton()
        loadFilesBox.text = "Load files"
        loadFilesBox.collapsed = True
        loadLayout.addWidget(loadFilesBox)
        loadFilesForm = qt.QFormLayout(loadFilesBox)

        self.loadCTBtn = qt.QPushButton("Load CT...")
        self.loadCTBtn.clicked.connect(self._onLoadCT)
        loadFilesForm.addRow(self.loadCTBtn)

        self.loadBiomodelBtn = qt.QPushButton("Load Biomodel...")
        self.loadBiomodelBtn.clicked.connect(self._onLoadBiomodel)
        loadFilesForm.addRow(self.loadBiomodelBtn)

        self.loadMarkerBtn = qt.QPushButton("Load Marker...")
        self.loadMarkerBtn.clicked.connect(self._onLoadMarker)
        loadFilesForm.addRow(self.loadMarkerBtn)

        self.createStarBallsBtn = qt.QPushButton("Create StarBalls...")
        self.createStarBallsBtn.clicked.connect(self._onCreateStarBalls)
        loadFilesForm.addRow(self.createStarBallsBtn)

        #Scene nodes
        sceneBox = ctk.ctkCollapsibleButton()
        sceneBox.text = "Select scene nodes"
        sceneBox.collapsed = True
        loadLayout.addWidget(sceneBox)
        sceneForm = qt.QFormLayout(sceneBox)

        self.volumeSelector = slicer.qMRMLNodeComboBox()
        self.volumeSelector.nodeTypes              = ["vtkMRMLScalarVolumeNode"]
        self.volumeSelector.selectNodeUponCreation = True
        self.volumeSelector.addEnabled             = False
        self.volumeSelector.removeEnabled          = False
        self.volumeSelector.noneEnabled            = True
        self.volumeSelector.setMRMLScene(slicer.mrmlScene)
        sceneForm.addRow("CT Volume:", self.volumeSelector)

        self.ballsSelector = slicer.qMRMLNodeComboBox()
        self.ballsSelector.nodeTypes              = ["vtkMRMLMarkupsFiducialNode"]
        self.ballsSelector.selectNodeUponCreation = False
        self.ballsSelector.addEnabled             = False
        self.ballsSelector.removeEnabled          = False
        self.ballsSelector.noneEnabled            = True
        self.ballsSelector.setMRMLScene(slicer.mrmlScene)
        sceneForm.addRow("IR Spheres (StarBalls):", self.ballsSelector)

        self.biomodelSelector = slicer.qMRMLNodeComboBox()
        self.biomodelSelector.nodeTypes              = ["vtkMRMLSegmentationNode"]
        self.biomodelSelector.selectNodeUponCreation = False
        self.biomodelSelector.addEnabled             = False
        self.biomodelSelector.removeEnabled          = False
        self.biomodelSelector.noneEnabled            = True
        self.biomodelSelector.setMRMLScene(slicer.mrmlScene)
        sceneForm.addRow("Biomodel (mask):", self.biomodelSelector)

        self.markerModelSelector = slicer.qMRMLNodeComboBox()
        self.markerModelSelector.nodeTypes              = ["vtkMRMLModelNode"]
        self.markerModelSelector.selectNodeUponCreation = False
        self.markerModelSelector.addEnabled             = False
        self.markerModelSelector.removeEnabled          = False
        self.markerModelSelector.noneEnabled            = True
        self.markerModelSelector.setMRMLScene(slicer.mrmlScene)
        sceneForm.addRow("Reference marker (STL):", self.markerModelSelector)

        self.centroidLabel = qt.QLabel("—")
        self.centroidLabel.setStyleSheet("font-family: monospace; font-size: 11px; color: gray;")
        sceneForm.addRow("Centroid of spheres in IR:", self.centroidLabel)

        readBtn = qt.QPushButton("↺  Read sphere position")
        readBtn.clicked.connect(self._readBalls)
        sceneForm.addRow(readBtn)

        #CONNECTION
        connBox = ctk.ctkCollapsibleButton()
        connBox.text = "CONNECTION"
        connBox.collapsed = True
        setupLayout.addWidget(connBox)
        connForm = qt.QFormLayout(connBox)

        # PLUS/OpenIGTLink - todavía pendiente
        connNote = qt.QLabel("PLUS / OpenIGTLink connection — coming soon.")
        connNote.setStyleSheet("color: gray; font-size: 11px;")
        connForm.addRow(connNote)

        # Surgeon display - ventana secundaria con las 3 vistas duplicadas
        self._secondaryWindow = None   # referencia a la ventana, None = no abierta
        self.surgeonDisplayBtn = qt.QPushButton("🖥  Open surgeon display")
        self.surgeonDisplayBtn.clicked.connect(self._onToggleSurgeonDisplay)
        connForm.addRow(self.surgeonDisplayBtn)

        surgeonNote = qt.QLabel(
            "Opens a secondary window mirroring the three slice views. "
            "If a second monitor is detected it launches fullscreen there."
        )
        surgeonNote.setStyleSheet("color: gray; font-size: 11px;")
        surgeonNote.setWordWrap(True)
        connForm.addRow(surgeonNote)

        #Visualizador de las coordenadas - por ahora es un input manual
        starGroup = qt.QGroupBox("COORDINATE VISUALIZER")
        starGroup.setSizePolicy(qt.QSizePolicy.Preferred, qt.QSizePolicy.Fixed)
        starNote = qt.QLabel("Camera coordinates of the reference marker.")
        starForm = qt.QFormLayout(starGroup)
        layout.addWidget(starGroup)

        self.starSpins = self._makeXYZSpins()
        starForm.addRow("X (mm):", self.starSpins[0])
        starForm.addRow("Y (mm):", self.starSpins[1])
        starForm.addRow("Z (mm):", self.starSpins[2])   
        starNote.setStyleSheet("color: gray; font-size: 11px;")
        starNote.setWordWrap(True)
        starForm.addRow(starNote)
        
        penNote = qt.QLabel("Camera coordinates of the instrument.")

        self.penSpins = self._makeXYZSpins()
        starForm.addRow("X (mm):", self.penSpins[0])
        starForm.addRow("Y (mm):", self.penSpins[1])
        starForm.addRow("Z (mm):", self.penSpins[2])
        penNote.setStyleSheet("color: gray; font-size: 11px;")
        penNote.setWordWrap(True)
        starForm.addRow(penNote)

        calcBtn = qt.QPushButton("Calculate instrument position in CT")
        calcBtn.setStyleSheet(
            "background: #2980b9; color: white; font-size: 14px;"
            " padding: 10px; border-radius: 4px;"
        )
        calcBtn.clicked.connect(self._calculate)
        starForm.addRow(calcBtn)

        self.penCtLabel = qt.QLabel("—")
        self.penCtLabel.setStyleSheet(
            "font-family: monospace; font-size: 13px; font-weight: bold;"
        )
        starForm.addRow("Instrument in CT (RAS):", self.penCtLabel)

        self.errorLabel = qt.QLabel("")
        self.errorLabel.setStyleSheet("color: #e74c3c; font-size: 11px;")
        self.errorLabel.setWordWrap(True)
        starForm.addRow(self.errorLabel)  
        

        # ── 4. Simulación ─────────────────────────────────────────────
        simBox = ctk.ctkCollapsibleButton()
        simBox.text = "MOVEMENT SIMULATION"
        layout.addWidget(simBox)
        simLayout = qt.QVBoxLayout(simBox)
        simBox.collapsed = True

        # Modo
        modeRow = qt.QHBoxLayout()
        modeRow.addWidget(qt.QLabel("Mode:"))
        self.modeCombo = qt.QComboBox()
        self.modeCombo.addItems([
            "Random in the CT",
            "Linear Trajectory R",
            "Linear Trajectory A",
            "Linear Trajectory S",
            "Axial spiral",
        ])
        modeRow.addWidget(self.modeCombo)
        simLayout.addLayout(modeRow)

        # Intervalo
        intervalRow = qt.QHBoxLayout()
        intervalRow.addWidget(qt.QLabel("Each (ms):"))
        self.intervalSpin = qt.QSpinBox()
        self.intervalSpin.setRange(100, 5000)
        self.intervalSpin.setValue(500)
        self.intervalSpin.setSingleStep(100)
        intervalRow.addWidget(self.intervalSpin)
        simLayout.addLayout(intervalRow)

        # Botón start/stop
        self.simBtn = qt.QPushButton("▶  Start Simulation")
        self.simBtn.setCheckable(True)
        self.simBtn.setStyleSheet(
            "QPushButton { background: #27ae60; color: white; font-size: 13px;"
            " padding: 8px; border-radius: 4px; }"
            "QPushButton:checked { background: #e74c3c; }"
        )
        self.simBtn.toggled.connect(self._onSimToggle)
        simLayout.addWidget(self.simBtn)

        simNote = qt.QLabel(
            "The simulation fixes the marker on the tracker's origin and moves\n"
            "the instrument in positions within the CT."
        )
        simNote.setStyleSheet("color: gray; font-size: 11px;")
        simNote.setWordWrap(True)
        simLayout.addWidget(simNote)

        
        
        #Extras, por ahora si queremos quitar algun modelo o algo
        opacityBox = ctk.ctkCollapsibleButton()
        opacityBox.text = "TOGGLES"
        opacityBox.collapsed = True
        layout.addWidget(opacityBox)
        opacityForm = qt.QFormLayout(opacityBox)
        visLabel = qt.QLabel("Visibility trackers.")
        visLabel.setStyleSheet("color: gray; font-size: 11px; margin-left: 2px;")
        opacityForm.addRow(visLabel, qt.QLabel(""))

        self.modelOpacitySlider = ctk.ctkSliderWidget()
        self.modelOpacitySlider.minimum = 0
        self.modelOpacitySlider.maximum = 1
        self.modelOpacitySlider.value = 1
        self.modelOpacitySlider.singleStep = 0.05
        opacityForm.addRow("Biomodel:", self.modelOpacitySlider)
        self.modelOpacitySlider.valueChanged.connect(self._onBiomodelOpacity)
        
        self.markerOpacitySlider = ctk.ctkSliderWidget()
        self.markerOpacitySlider.minimum = 0
        self.markerOpacitySlider.maximum = 1
        self.markerOpacitySlider.value = 1
        self.markerOpacitySlider.singleStep = 0.05
        opacityForm.addRow("Reference Marker:", self.markerOpacitySlider)
        self.markerOpacitySlider.valueChanged.connect(self._onMarkerOpacity)

        self.ctOpacitySlider = ctk.ctkSliderWidget()
        self.ctOpacitySlider.minimum = 0
        self.ctOpacitySlider.maximum = 1
        self.ctOpacitySlider.value = 1
        self.ctOpacitySlider.singleStep = 0.05
        opacityForm.addRow("CT:", self.ctOpacitySlider)
        self.ctOpacitySlider.valueChanged.connect(self._onCTOpacity)
        
        layout.addStretch()

    # ── Helpers ───────────────────────────────────────────────────────────

    def _makeXYZSpins(self):
        spins = []
        for _ in range(3):
            s = qt.QDoubleSpinBox()
            s.setRange(-1000.0, 1000.0)
            s.setValue(0.0)
            s.setSingleStep(1.0)
            s.setDecimals(2)
            s.setSuffix(" mm")
            spins.append(s)
        return spins

    def _getXYZ(self, spins):
        return np.array([s.value for s in spins])

    def _readBalls(self):
        """Lee el centroide actual de las bolas y lo muestra."""
        balls = self.ballsSelector.currentNode()
        if balls is None:
            self.centroidLabel.setText("(no markup selected)")
            return
        if balls.GetNumberOfControlPoints() < 3:
            self.centroidLabel.setText("⚠ You need 3 control points")
            return
        try:
            centroid = self.logic.getCentroidInCT(balls)
            self.centroidLabel.setText(
                f"R={centroid[0]:+.1f}  A={centroid[1]:+.1f}  S={centroid[2]:+.1f} mm"
            )
        except Exception as e:
            self.centroidLabel.setText(f"Error: {e}")

    def _onSimToggle(self, checked):
        if checked:
            self.simBtn.setText("■  Stop simulation")
            balls = self.ballsSelector.currentNode()
            if balls is None or balls.GetNumberOfControlPoints() < 3:
                slicer.util.warningDisplay("Select StarBalls before simulating.")
                self.simBtn.setChecked(False)
                return
            self.logic.resetTrajectory()
            self._timer.start(self.intervalSpin.value)
        else:
            self.simBtn.setText("▶  Start simulation")
            self._timer.stop()

    def _oneShot(self):
        balls = self.ballsSelector.currentNode()
        if balls is None or balls.GetNumberOfControlPoints() < 3:
            slicer.util.warningDisplay("Select StarBalls before simulating.")
            return
        self._applySimulatedPosition(balls, self.modeCombo.currentIndex)

    def _tick(self):
        balls = self.ballsSelector.currentNode()
        if balls is None:
            self._timer.stop()
            return
        self._applySimulatedPosition(balls, self.modeCombo.currentIndex)

    def _applySimulatedPosition(self, balls, mode):
        """
        Genera coordenadas simuladas del marcador y el lápiz en el tracker
        según el modo seleccionado y las aplica directamente.
        """
        vol = self.volumeSelector.currentNode()
        bounds = [0] * 6
        if vol:
            vol.GetRASBounds(bounds)
        else:
            bounds = [-150, 150, -150, 150, -150, 150]

        star_xyz = self._getXYZ(self.starSpins)
        pen_xyz  = self.logic.nextPosition(mode, bounds)

        # Actualizar spinboxes del lápiz
        for spin, val in zip(self.penSpins, pen_xyz):
            spin.setValue(float(val))

        # Calcular y saltar
        self.errorLabel.setText("")
        try:
            pen_ct = self.logic.computePenInCT(balls, star_xyz, pen_xyz)
            self.penCtLabel.setText(
                f"R={pen_ct[0]:+.1f}  A={pen_ct[1]:+.1f}  S={pen_ct[2]:+.1f}"
            )
            self.logic.jumpToRAS(pen_ct)
        except Exception as e:
            self.errorLabel.setText(f"⚠ Error: {e}")
    
    def _onLoadCT(self):
        path = qt.QFileDialog.getOpenFileName(
            None, "Load CT", "", "Volume files (*.nrrd *.nii *.nii.gz *.mha *.mhd *.dcm)"
        )
        if path:
            slicer.util.loadVolume(path)

    def _onLoadBiomodel(self):
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Biomodel", "", "Model files (*.stl *.vtk *.obj *.ply)"
        )
        if path:
            slicer.util.loadSegmentation(path)

    def _onLoadMarker(self):
        path = qt.QFileDialog.getOpenFileName(
            None, "Load Marker STL", "", "Model files (*.stl *.vtk *.obj)"
        )
        if path:
            slicer.util.loadModel(path)

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
        node = self.biomodelSelector.currentNode()
        if node is None:
            return
        node.GetDisplayNode().SetOpacity(value)

    def _onMarkerOpacity(self, value):
        node = self.markerModelSelector.currentNode()
        if node is None:
            return
        node.GetDisplayNode().SetOpacity(value)

    def _onCTOpacity(self, value):
        vol = self.volumeSelector.currentNode()
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
            self.surgeonDisplayBtn.setText("🖥  Open surgeon display")
            return

        # Crear ventana nueva
        self._secondaryWindow = self._buildSurgeonWindow()
        self.surgeonDisplayBtn.setText("✖  Close surgeon display")

    # ── Handler principal ─────────────────────────────────────────────────

    def _calculate(self):
        self.errorLabel.setText("")

        balls = self.ballsSelector.currentNode()
        if balls is None:
            self.errorLabel.setText(
                "⚠ Select the StarBalls list (3 markups of IR spheres)."
            )
            return
        if balls.GetNumberOfControlPoints() < 3:
            self.errorLabel.setText(
                "⚠ StarBalls needs exactly 3 control points."
            )
            return

        star_xyz = self._getXYZ(self.starSpins)
        pen_xyz  = self._getXYZ(self.penSpins)

        try:
            pen_ct = self.logic.computePenInCT(balls, star_xyz, pen_xyz)
        except Exception as e:
            self.errorLabel.setText(f"⚠ Error: {e}")
            return

        self.penCtLabel.setText(
            f"R={pen_ct[0]:+.1f}  A={pen_ct[1]:+.1f}  S={pen_ct[2]:+.1f}"
        )
        self._readBalls()
        self.logic.jumpToRAS(pen_ct)
        
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
        screens = qt.QGuiApplication.screens()
        if len(screens) > 1:
            # Hay un 2º monitor → fullscreen allí
            secondaryScreen = screens[1]
            geom = secondaryScreen.geometry()
            win.move(geom.x(), geom.y())
            win.showFullScreen()
        else:
            # Solo 1 monitor → ventana normal, tamaño razonable
            win.resize(1200, 400)
            win.show()

        return win

    def cleanup(self):
        """Se llama cuando Slicer descarga el módulo. Cerramos la ventana
        secundaria si estuviera abierta para evitar widgets huérfanos."""
        if getattr(self, "_secondaryWindow", None) is not None:
            self._secondaryWindow.close()
            self._secondaryWindow = None
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

    def computePenInCT(self, ballsNode, star_xyz, pen_xyz):
        """
        ballsNode : vtkMRMLMarkupsFiducialNode — StarBalls con 3 puntos
        star_xyz  : np.array [x,y,z] — posición marcador en tracker (simulada)
        pen_xyz   : np.array [x,y,z] — posición lápiz en tracker (simulada)

        Devuelve pen_ct: np.array [x,y,z] en espacio CT (RAS, mm)
        """
        centroid       = self.getCentroidInCT(ballsNode)
        T_star2ct      = self._translation(centroid)
        T_tracker2star = self._translation(star_xyz)
        T_tracker2pen  = self._translation(pen_xyz)

        T_ct_pen = (
            np.linalg.inv(T_star2ct)
            @ np.linalg.inv(T_tracker2star)
            @ T_tracker2pen
        )
        return T_ct_pen[:3, 3]

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

    TRAJECTORY_STEPS = 60

    def resetTrajectory(self):
        self._step = 0

    def nextPosition(self, mode, bounds):
        """
        Genera la siguiente posición objetivo dentro del CT según el modo.
        bounds = [rMin, rMax, aMin, aMax, sMin, sMax]
        """
        if not hasattr(self, "_step"):
            self._step = 0

        def inset(lo, hi, pct=0.10):
            m = (hi - lo) * pct
            return lo + m, hi - m

        rLo, rHi = inset(bounds[0], bounds[1])
        aLo, aHi = inset(bounds[2], bounds[3])
        sLo, sHi = inset(bounds[4], bounds[5])
        rMid = (rLo + rHi) / 2
        aMid = (aLo + aHi) / 2
        sMid = (sLo + sHi) / 2
        t = self._step / self.TRAJECTORY_STEPS

        if mode == 0:
            return np.array([
                random.uniform(rLo, rHi),
                random.uniform(aLo, aHi),
                random.uniform(sLo, sHi),
            ])
        elif mode == 1:
            self._advanceStep()
            return np.array([rLo + (rHi - rLo) * t, aMid, sMid])
        elif mode == 2:
            self._advanceStep()
            return np.array([rMid, aLo + (aHi - aLo) * t, sMid])
        elif mode == 3:
            self._advanceStep()
            return np.array([rMid, aMid, sLo + (sHi - sLo) * t])
        elif mode == 4:
            angle  = t * 4 * math.pi
            radius = min(rHi - rMid, aHi - aMid) * t
            self._advanceStep()
            return np.array([
                rMid + radius * math.cos(angle),
                aMid + radius * math.sin(angle),
                sLo  + (sHi - sLo) * t,
            ])
        return np.array([rMid, aMid, sMid])

    def _advanceStep(self):
        self._step = (self._step + 1) % (self.TRAJECTORY_STEPS + 1)

    def _translation(self, xyz):
        T = np.eye(4)
        T[:3, 3] = xyz
        return T


# ─────────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────────

class CTNavigatorTest(ScriptedLoadableModuleTest):

    def setUp(self):
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        self.setUp()
        self.test_IdentityChain()
        self.test_TranslationChain()
        self.test_RelativeOffset()

    def _makeBalls(self, offset_xyz):
        """
        Crea un nodo StarBalls con 3 markups en las coordenadas locales
        conocidas, desplazados por offset_xyz (simula la transform).
        """
        node = slicer.mrmlScene.AddNewNodeByClass(
            "vtkMRMLMarkupsFiducialNode", "StarBalls"
        )
        local = np.array([
            [  0.0, -55.0, 8.5],
            [ 35.0,   5.0, 8.5],
            [-35.0,   5.0, 8.5],
        ])
        for p in local:
            node.AddControlPoint(*(p + offset_xyz))
        return node

    def test_IdentityChain(self):
        """Marcador y lápiz en origen → lápiz en CT = offset de las bolas."""
        self.delayDisplay("Test: cadena identidad...")
        logic = CTNavigatorLogic()
        balls = self._makeBalls([0.0, 0.0, 0.0])
        pen_ct = logic.computePenInCT(
            balls,
            star_xyz=np.array([0.0, 0.0, 0.0]),
            pen_xyz =np.array([10.0, 0.0, 0.0]),
        )
        # centroide local = (0, -15, 8.5), pen offset = (10,0,0)
        # → pen en CT = centroide + (10,0,0) = (10, -15, 8.5)
        expected = np.array([10.0, -15.0, 8.5])
        np.testing.assert_allclose(pen_ct, expected, atol=1e-3)
        self.delayDisplay("✓ test_IdentityChain OK")

    def test_TranslationChain(self):
        """
        Marcador desplazado 100mm en R en CT.
        Marcador en tracker en (50,0,0), lápiz en (60,0,0).
        Offset lápiz-marcador = 10mm en R → lápiz en CT en centroide + 10mm.
        """
        self.delayDisplay("Test: traslación simple...")
        logic  = CTNavigatorLogic()
        balls  = self._makeBalls([100.0, 0.0, 0.0])
        pen_ct = logic.computePenInCT(
            balls,
            star_xyz=np.array([50.0, 0.0, 0.0]),
            pen_xyz =np.array([60.0, 0.0, 0.0]),
        )
        centroid = np.array([0.0, -15.0, 8.5]) + np.array([100.0, 0.0, 0.0])
        expected = centroid + np.array([10.0, 0.0, 0.0])
        np.testing.assert_allclose(pen_ct, expected, atol=1e-3)
        self.delayDisplay("✓ test_TranslationChain OK")

    def test_RelativeOffset(self):
        """
        Lo que importa es el offset relativo pen_xyz - star_xyz.
        Offset = (5, 3, 2) → en CT: centroide + offset.
        """
        self.delayDisplay("Test: offset relativo...")
        logic  = CTNavigatorLogic()
        balls  = self._makeBalls([50.0, 0.0, 0.0])
        pen_ct = logic.computePenInCT(
            balls,
            star_xyz=np.array([100.0, 0.0, 0.0]),
            pen_xyz =np.array([105.0, 3.0, 2.0]),
        )
        centroid = np.array([0.0, -15.0, 8.5]) + np.array([50.0, 0.0, 0.0])
        expected = centroid + np.array([5.0, 3.0, 2.0])
        np.testing.assert_allclose(pen_ct, expected, atol=1e-3)
        self.delayDisplay("✓ test_RelativeOffset OK")