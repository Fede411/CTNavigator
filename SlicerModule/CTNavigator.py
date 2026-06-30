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
        self.trackBtn = qt.QPushButton("▶  Start tracking")
        self.trackBtn.setCheckable(True)
        self.trackBtn.setStyleSheet(
            "QPushButton { background: #27ae60; color: white; font-size: 13px;"
            " padding: 8px; border-radius: 4px; }"
            "QPushButton:checked { background: #e74c3c; }"
        )
        self.trackBtn.toggled.connect(self._onTrackToggle)
        connForm.addRow(self.trackBtn)

        connNote = qt.QLabel(
            "Reads MarkerToTracker and ToolToTracker from the OpenIGTLink\n"
            "connector and updates the instrument position in the CT in real time."
        )
        connNote.setStyleSheet("color: gray; font-size: 11px;")
        connNote.setWordWrap(True)
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

        # Posición del instrumento en el CT (la escribe el tracking en _onToolMoved)
        self.penCtLabel = qt.QLabel("—")
        self.penCtLabel.setStyleSheet(
            "font-family: monospace; font-size: 13px; font-weight: bold;"
        )
        connForm.addRow("Instrument in CT (RAS):", self.penCtLabel)

        self.errorLabel = qt.QLabel("")
        self.errorLabel.setStyleSheet("color: #e74c3c; font-size: 11px;")
        self.errorLabel.setWordWrap(True)
        connForm.addRow(self.errorLabel)

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

    def _onTrackToggle(self, checked):
        if checked:
            # Verificar StarBalls
            balls = self.ballsSelector.currentNode()
            if balls is None or balls.GetNumberOfControlPoints() < 3:
                slicer.util.warningDisplay("Select StarBalls (3 points) before tracking.")
                self.trackBtn.setChecked(False)
                return

            # Verificar que el node del instrumento ya llega
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToTracker")
            if toolNode is None:
                slicer.util.warningDisplay(
                    "ToolToTracker not found. Make sure the OpenIGTLink "
                    "connector is active and the Kinect is detecting the instrument."
                )
                self.trackBtn.setChecked(False)
                return

            # Suscribir el observer al node del instrumento
            self.trackBtn.setText("■  Stop tracking")
            self._toolObserverTag = toolNode.AddObserver(
                slicer.vtkMRMLTransformNode.TransformModifiedEvent,
                self._onToolMoved
            )
        else:
            self.trackBtn.setText("▶  Start tracking")
            # Quitar el observer
            toolNode = slicer.mrmlScene.GetFirstNodeByName("ToolToTracker")
            if toolNode is not None and hasattr(self, "_toolObserverTag"):
                toolNode.RemoveObserver(self._toolObserverTag)


    def _onToolMoved(self, caller, event):
        balls = self.ballsSelector.currentNode()
        if balls is None:
            return

        T_star2tracker = self.logic._readTransform("MarkerToTracker")
        T_pen2tracker  = self.logic._readTransform("ToolToTracker")

        if T_star2tracker is None or T_pen2tracker is None:
            return

        try:
            T_pen2ct = self.logic.computePenInCT(balls, T_star2tracker, T_pen2tracker)
            pos = T_pen2ct[:3, 3]
            self.penCtLabel.setText(
                f"R={pos[0]:+.1f}  A={pos[1]:+.1f}  S={pos[2]:+.1f}"
            )
            self.logic.jumpToRAS(pos)
        except Exception as e:
            self.errorLabel.setText(f"⚠ Error: {e}")
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