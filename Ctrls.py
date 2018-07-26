from pymel.core import *


class Ctrl(object):
    """
    Builds baseline attributes for our child classes to use
    """

    def __init__(self):

        self.ctrl = None
        self.createCtrlShaders()

    def getCtrlInfo(self, name):

        '''Gets our ctrl information ready to go through other class methods'''

        self.ctrlShape = self.ctrl[0].getChildren()
        self.ctrlLoc = spaceLocator(n=name + '_CTRL_BUFFER')
        select(d=True)
        self.ctrlOffset = group(n=name + '_CTRL_TFM', em=1)
        select(d=True)
        self.ctrl[0].setParent(self.ctrlLoc)
        self.ctrlLoc.setParent(self.ctrlOffset)

    def createCtrlShaders(self):

        '''creates five color shaders for our ctrl setup'''

        if objExists('blue_Shader') and objExists('orange_Shader'):
            self.bShader = PyNode('blue_Shader')
            self.rShader = PyNode('red_Shader')
            self.gShader = PyNode('green_Shader')
            self.cShader = PyNode('cyan_Shader')
            self.oShader = PyNode('orange_Shader')
        else:
            self.bShader = shadingNode("lambert", n='blue_Shader', asShader=True)
            self.bShader.color.set(0, 0, 1)
            self.bShader.transparency.set(.9, .9, .9)
            self.bShader.incandescence.set(0, 0, .2)

            self.rShader = shadingNode("lambert", n='red_Shader', asShader=True)
            self.rShader.color.set(1, 0, 0)
            self.rShader.transparency.set(.9, .9, .9)
            self.rShader.incandescence.set(.2, 0, 0)

            self.gShader = shadingNode("lambert", n='green_Shader', asShader=True)
            self.gShader.color.set(0, 1, 0)
            self.gShader.transparency.set(.9, .9, .9)
            self.gShader.incandescence.set(0, .2, 0)

            self.cShader = shadingNode("lambert", n='cyan_Shader', asShader=True)
            self.cShader.color.set(0, .7, 1)
            self.cShader.transparency.set(.9, .9, .9)
            self.cShader.incandescence.set(.05, .17, .17)

            self.oShader = shadingNode("lambert", n='orange_Shader', asShader=True)
            self.oShader.color.set(1, .25, 0)
            self.oShader.transparency.set(.9, .9, .9)
            self.oShader.incandescence.set(.2, .12, 0)

        select(d=True)

    def setCtrlColor(self, name):

        '''
        Sets the color of the a controler based on the naming prefix
        '''

        select(self.ctrlShape[0])
        self.ctrlShape[0].overrideEnabled.set(True)

        if name.startswith('L_'):
            self.ctrlShape[0].ovc.set(15)
            hyperShade(self.ctrlShape[0], assign=self.bShader)

        elif name.startswith('R_'):
            self.ctrlShape[0].ovc.set(13)
            hyperShade(self.ctrlShape[0], assign=self.rShader)

        elif name.startswith('Master') or name.startswith('Root_'):
            self.ctrlShape[0].ovc.set(18)
            hyperShade(self.ctrlShape[0], assign=self.cShader)

        elif name.startswith('Spine') or name.startswith('Pelvis') or name.startswith('neck') or name.startswith('headRoot'):
            self.ctrlShape[0].ovc.set(14)
            hyperShade(self.ctrlShape[0], assign=self.gShader)

        else:
            self.ctrlShape[0].ovc.set(14)
            hyperShade(self.ctrlShape[0], assign=self.oShader)

        sets('initialShadingGroup', rm=self.ctrlShape[0])
        select(d=True)

    def setRotation(self, rotate=(0, 0, 0)):

        '''
        Rotate the Ctrl and freezes the tranformation so we have a viusally correct controll
        '''
        self.ctrl[0].rotate.set(rotate)
        makeIdentity(self.ctrl[0], a=True, r=True, pn=True)
        select(d=True)

    def setTranslation(self, translate=(0, 0, 0)):

        '''
        Translates the Ctrl and freezes the tranformation so we have a viusally correct controll
        '''
        self.ctrl[0].translate.set(translate)
        makeIdentity(self.ctrl[0], a=True, t=True, pn=True)
        select(d=True)

    def setScale(self, scale=(0, 0, 0)):

        '''
        Scales the Ctrl and freezes the tranformation so we have a viusally correct controll
        '''
        self.ctrl[0].scale.set(scale)
        makeIdentity(self.ctrl[0], a=True, s=True, pn=True)
        select(d=True)
    def setParent(self, parent=None, maintainOffset=False):

        '''
        Will move the control to the parent if maintaint offset is left default. Than it will be parented under parent argument
        '''

        delete(parentConstraint(parent, self.ctrlOffset, mo=maintainOffset))
        self.ctrlOffset.setParent(parent)
        select(d=True)
        
    def setPosition(self, parent=None, maintainOffset=False):

        '''
        Will move the control to the parent if maintaint offset is left default. Than it will be parented under parent argument
        '''

        delete(parentConstraint(parent, self.ctrlOffset, mo=maintainOffset))

    def setChildIk(self, ikHandle=None, maintainOffset=False):

        '''
        This is an Ik specfic constrain process. It will get the Iks in place with correct orientation. The ctrl shape may still need
        to be rotated with the "setRotation" function.
        '''

        endJoint = ikHandle.getEndEffector().getSiblings()
        armIk = endJoint[0].jointOrientX.get() == 0.0 and endJoint[0].jointOrientY.get() == 0.0 and endJoint[
            0].jointOrientZ.get() == 0.0

        if armIk:
            if self.ctrlOffset.startswith('R_'):
                self.ctrlOffset.scale.set(1, -1, -1)
            delete(parentConstraint(endJoint[0], self.ctrlOffset, mo=maintainOffset))
        else:
            delete(parentConstraint(ikHandle, self.ctrlOffset, mo=maintainOffset))

        ikHandle.setParent(self.ctrl[0])
        select(d=True)

    def setDriven(self, driven=None, child = False):

        '''
        set The driven object to the driver MAY NOT BE NEEDED
        '''

        delete(parentConstraint(driven, self.ctrlOffset, mo=False))
        if child ==False:
            parentConstraint(self.ctrl[0], driven)
        else:
            parent(driven,self.ctrl[0])
        select(d=True)

    def insert(self, driven=None):
        '''wip'''
        pass


class sphereCtrl(Ctrl, object):
    '''
    creates sphere ctrl the user will use the parent CTRL class functions for manipulation
    '''

    def __init__(self, name='', **kwargs):
        super(sphereCtrl, self).__init__()
        self.ctrl = sphere(n=name + '_CTRL',ax=(0, 1, 0), **kwargs)

        super(sphereCtrl, self).getCtrlInfo(name)
        super(sphereCtrl, self).setCtrlColor(name)


class cylinderCtrl(Ctrl, object):
    '''
    creates cylinder/box ctrl the user will use the parent CTRL class functions for manipulation
    '''

    def __init__(self, name='',  **kwargs):
        super(cylinderCtrl, self).__init__()
        self.ctrl = cylinder(n=name + '_CTRL',ax=(0, 1, 0),**kwargs)

        super(cylinderCtrl, self).getCtrlInfo(name)
        super(cylinderCtrl, self).setCtrlColor(name)


class torusCtrl(Ctrl, object):
    '''
    creates torus ctrl the user will use the parent CTRL class functions for manipulation
    '''

    def __init__(self, name='', **kwargs):
        super(torusCtrl, self).__init__()
        self.ctrl = torus(n=name + '_CTRL',ax=(0, 1, 0), **kwargs)

        super(torusCtrl, self).getCtrlInfo(name)
        super(torusCtrl, self).setCtrlColor(name)


class coneCtrl(Ctrl, object):
    '''
    creates cone ctrl the user will use the parent CTRL class functions for manipulation
    '''

    def __init__(self, name='', **kwargs ):
        super(coneCtrl, self).__init__()
        self.ctrl = cone(n=name + '_CTRL',ax=(0, 1, 0),**kwargs)

        super(coneCtrl, self).getCtrlInfo(name)
        super(coneCtrl, self).setCtrlColor(name)


if __name__ == "__main__":
    list = ['L_Arm', 'R_Arm', 'Spine', 'Master_Please', 'Root_Stuff', 'Spine', 'Pelvis', 'Shit']
    for i in list:
        hi = torusCtrl(name=i)