from PxAutorig import AutoRig as AR
import PxAutorig.AutoRig


reload(AR)
# from PySide2 import QtWidgets, QtCore, QtGui
from PxAutorig.Qt import QtWidgets, QtCore, QtGui
from pymel.core import *




# from see import see
 

class PxAutoRigUI(QtWidgets.QDialog):

    def __init__(self):
        super(PxAutoRigUI, self).__init__()

        self.setWindowTitle('Px Auto Rig')
        self.Ar = AR.PxAutoRig()
        self.buildUI()
        self.populateUI()

    def buildUI(self):
        layout = QtWidgets.QGridLayout(self)

        armTargetsBtn = QtWidgets.QPushButton('Create Arm Targets')
        legTargetsBtn = QtWidgets.QPushButton('Create Leg Targets')
        spineTargetsBtn = QtWidgets.QPushButton('Create Spine Targets')
        gobalHeadTargetsBtn = QtWidgets.QPushButton('Create Gobal Head Targets')
        handTargetsBtn = QtWidgets.QPushButton('Create Hand Targets')

        createArmChainsBtn = QtWidgets.QPushButton('Create Arm Chains')
        createLeftLegBtn = QtWidgets.QPushButton('Create Left Leg')
        createSpineChainBtn = QtWidgets.QPushButton('Create Spine Chain')
        createGlobalHeadChainBtn = QtWidgets.QPushButton('Create Global Head Chain')
        createLeftHandBtn = QtWidgets.QPushButton('Create Left Hand')

        createArmDrvsBtn = QtWidgets.QPushButton('Create Arm Controls')
        createLegDrvsBtn = QtWidgets.QPushButton('Create Leg Controls')
        createHandDrvsBtn = QtWidgets.QPushButton('Create Hand Controls')
        createSpineDrvsBtn = QtWidgets.QPushButton('Create Spine Controls')

        armTargetsBtn.clicked.connect(self.Ar.createArmTargets)
        legTargetsBtn.clicked.connect(self.Ar.createLegTargets)
        spineTargetsBtn.clicked.connect(self.Ar.createSpineTargets)
        gobalHeadTargetsBtn.clicked.connect(self.Ar.createGobalHeadTargets)
        handTargetsBtn.clicked.connect(self.Ar.createHandTargets)

        createArmChainsBtn.clicked.connect(self.Ar.createLeftArmChain)
        createLeftLegBtn.clicked.connect(self.Ar.createLeftLeg)
        createGlobalHeadChainBtn.clicked.connect(self.Ar.createGlobalHeadChain)
        createLeftHandBtn.clicked.connect(self.Ar.createLeftHand)
        createSpineChainBtn.clicked.connect(self.Ar.createSpineChain)

        createArmDrvsBtn.clicked.connect(self.Ar.createArmDrvs)
        createLegDrvsBtn.clicked.connect(self.Ar.createLegDrvs)
        createSpineDrvsBtn.clicked.connect(self.Ar.createSpineDrvs)
        createHandDrvsBtn.clicked.connect(self.Ar.createHandDrvs)

        layout.addWidget(armTargetsBtn, 0, 0)
        layout.addWidget(legTargetsBtn, 0, 1)
        layout.addWidget(spineTargetsBtn, 0, 2)
        layout.addWidget(handTargetsBtn, 0, 3)
        layout.addWidget(gobalHeadTargetsBtn, 0, 4)

        layout.addWidget(createArmChainsBtn, 1, 0)
        layout.addWidget(createLeftLegBtn, 1, 1)
        layout.addWidget(createSpineChainBtn, 1, 2)
        layout.addWidget(createLeftHandBtn, 1, 3)
        layout.addWidget(createGlobalHeadChainBtn, 1, 4)
        

        layout.addWidget(createArmDrvsBtn, 2, 0)
        layout.addWidget(createLegDrvsBtn, 2, 1)
        layout.addWidget(createSpineDrvsBtn, 2, 2)
        layout.addWidget(createHandDrvsBtn, 2,3)
        

    def populateUI(self):
        print 'populating it '


def showUI():
    ui = PxAutoRigUI()
    ui.show()
    return ui


ui = showUI()