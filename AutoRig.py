'''
    File name: PxAutoRig.py
    Author: Wayne Moodie & Alexander Wilson
    Date created: 5/31/2018
    Email : moodiewayne@gmail.com and alexanderwilson426@gmail.com
    Date last modified: 7/12/2018
    Python Version: 2.7
    Todo List : Start Facial Setups
				fix twist joint setup
				Clean up outliner
				add Global scale
				add Control Cleaner
				add mirror Hand motion and hide secondary controls
				Update UI


'''
from functools import wraps
from pymel.core import*
import PxAutorig.FuncUtils as Ut
import PxAutorig.Ctrls as Ct
reload(Ut)
reload(Ct)

def undoFunc(func):
    '''
    undo function decorator
    '''
    @wraps(func)
    def funcWrapper(*args, **kwargs):
        undoInfo(openChunk=True, chunkName=func.__name__)
        try:
            result = func(*args, **kwargs)
            undoInfo(closeChunk=True)
            return result
        except:
            undoInfo(closeChunk=True)
            if undoInfo(query=True, undoName=True) == func.__name__:
                undo()
            raise  # this doesn't raise the exception
    return funcWrapper

class PxAutoRig:

    '''A modular based autorig, with parrlell evaluation in mind'''

    def __init__(self, *args):
        self.ctrlCleanList = []
        self.createMasterCtrl()
        
#_________________________Create___DataNode__________________________________________________#

        if objExists('MetaData'):
            self.dataNode = PyNode('MetaData')
        else:
            '''creates a DataNode to store target placments for next iterataions'''
            self.dataNode = group(n = 'MetaData')
            Ut.lockScale(self.dataNode)
            Ut.lockRotate(self.dataNode)
            Ut.lockTranslate(self.dataNode)
            Ut.lockViz(self.dataNode)

    def updateDataNode(self, jntChainTransforms = None):

        '''Update the Targets positions on the metaDataGroup'''


        for trans in jntChainTransforms:
            pos = trans.getTranslation(w =True)
            i=0
            for axis in 'XYZ':
                setAttr(self.dataNode+'.'+trans[:-4]+axis ,pos[i])
                i+=1

    def populateData(self, attrList):

        '''checks if metaDataNode has given attributes. if it does not have that
        attribute it will populate it with the given attributes'''

        lockNode(self.dataNode, l =False)

        attrAddList = [attr[:-4]+axis for attr in attrList for axis in 'XYZ']

        for attr in attrAddList:
            if self.dataNode.hasAttr(attr) == True:
                pass
            else:
                addAttr(self.dataNode, ln= attr , at = 'double', k=True)

        lockNode(self.dataNode, l= True)


    def getPosVecData(self, posQuery):

        '''Query the position from the metaDataNode, and Return the list of pos vectors'''

        posVector = []
        posList = [self.dataNode.getAttr(pos+ax) for pos in posQuery for ax in 'XYZ']
        iter = 0
        i= [0,3]

        while iter != len(posList)/3:

            posVector.append(dt.Vector(posList[i[0]: i[1]]))
            iter +=1
            i[0]+=3
            i[1]+=3

        return posVector
#___________Create___Joint____targets________________________________________________________#


    @undoFunc
    def createMasterCtrl(self, *args):
        master = ['Master', 'subMaster']
        masterCtrlHeiarchy = []
        rad = 15
        for i in master:
            ctrl = Ct.torusCtrl(name=i,hr = .03 ,radius = rad)
            masterCtrlHeiarchy.append((ctrl.ctrlOffset, ctrl.ctrl[0]))
            rad-=2
        Ut.createCtrlHeiarchy(masterCtrlHeiarchy)
        self.masterCtrl = PyNode(masterCtrlHeiarchy[0][1])
        self.subMasterCtrl = PyNode(masterCtrlHeiarchy[1][1])

        attrList = ['L_Arm_IKFK_Switch','R_Arm_IKFK_Switch','L_Leg_IKFK_Switch','R_Leg_IKFK_Switch','Ctrl_Viz']
        for i in attrList:
            self.masterCtrl.addAttr(i ,k =True, max=1.0 , min =0.0)



    @undoFunc
    def createArmTargets(self, *args):

        armPosQuery = ['L_hand','L_wrist','L_elbow','L_shoulder','L_clavicle']
        if objExists(self.dataNode) and self.dataNode.hasAttr(armPosQuery[0]+'X') ==True :

            armPosVecs = self.getPosVecData(armPosQuery)

            print ('Arm TargetLocation Imported from metaData Node')
            self.handTarg      =  Ut.jointTarget(name = 'hand'     , pos = (armPosVecs[1]))
            self.wristTarg     =  Ut.jointTarget(name = 'wrist'    , pos = (armPosVecs[1]))
            self.elbowTarg     =  Ut.jointTarget(name = 'elbow'    , pos = (armPosVecs[2]))
            self.shoulderTarg  =  Ut.jointTarget(name = 'shoulder' , pos = (armPosVecs[3]))
            self.clavicleTarg  =  Ut.jointTarget(name = 'clavicle' , pos = (armPosVecs[4]))
        else:
            self.handTarg      =  Ut.jointTarget(name = 'hand'     , pos = (9.691,19.74,0.516))
            self.wristTarg     =  Ut.jointTarget(name = 'wrist'    , pos = (9.691,19.74,0.516))
            self.elbowTarg     =  Ut.jointTarget(name = 'elbow'    , pos = (6.963,23.282,-1.944))
            self.shoulderTarg  =  Ut.jointTarget(name = 'shoulder' , pos = (3.512,27.364,-1.866))
            self.clavicleTarg  =  Ut.jointTarget(name = 'clavicle' , pos = (0.611,27.985,-0.171))
              
        targetList = [self.handTarg, self.wristTarg, self.elbowTarg, self.shoulderTarg, self.clavicleTarg]
        self.armTargGrp =  Ut.jointTargViz(list = targetList, name = 'arm')
        print self.armTargGrp
        try:
            Ut.connectProtoHand(Driver = self.armTargGrp[-1],Driven = self.handTargGrp)
        except:
            pass
    @undoFunc
    def createLegTargets(self, *args):

        legPosQuery = ['L_toe','L_ball','L_ankle','L_knee','L_hip']
        if objExists(self.dataNode) and self.dataNode.hasAttr(legPosQuery[0]+'X') ==True :

            legPosVecs = self.getPosVecData(legPosQuery)
            print ('Leg TargetLocation Imported from metaData Node')
            self.toeTarg       =  Ut.jointTarget(name  = 'toe'      , pos = (legPosVecs[0]))
            self.ballTarg      =  Ut.jointTarget(name  = 'ball'     , pos = (legPosVecs[1]))
            self.ankleTarg     =  Ut.jointTarget(name  = 'ankle'    , pos = (legPosVecs[2]))
            self.kneeTarg      =  Ut.jointTarget(name  = 'knee'     , pos = (legPosVecs[3]))
            self.hipTarg       =  Ut.jointTarget(name  = 'hip'      , pos = (legPosVecs[4]))
        else:
            self.toeTarg       =  Ut.jointTarget(name  = 'toe'      , pos = (4.803,0,3.19))
            self.ballTarg      =  Ut.jointTarget(name  = 'ball'     , pos = (4.607,0,0.873))
            self.ankleTarg     =  Ut.jointTarget(name  = 'ankle'    , pos = (4.129,2.022,-1.514))
            self.kneeTarg      =  Ut.jointTarget(name  = 'knee'     , pos = (2.75,10.088,-0.382))
            self.hipTarg       =  Ut.jointTarget(name  = 'hip'      , pos = (1.659,16.856,-0.635))

        targetList = [self.toeTarg, self.ballTarg, self.ankleTarg, self.kneeTarg, self.hipTarg]
        self.legTargGrp= Ut.jointTargViz(list=targetList, name='leg')


    @undoFunc
    def createSpineTargets(self, *args):

        spinePosQuery = ['cSpine','bSpine','aSpine']
        if objExists(self.dataNode) and self.dataNode.hasAttr(spinePosQuery[0]+'X') ==True :

            spinePosVecs = self.getPosVecData(spinePosQuery)

            print ('Spine TargetLocation Imported from metaData Node')
            self.bSpineTarg    =  Ut.jointTarget(name  = 'bSpine'  , pos = (spinePosVecs[0]))
            self.aSpineTarg    =  Ut.jointTarget(name  = 'aSpine'  , pos = (spinePosVecs[1]))
        else:
            self.bSpineTarg    =  Ut.jointTarget(name  = 'bSpine'  , pos = (0,26.87,-1))
            self.aSpineTarg    =  Ut.jointTarget(name  = 'aSpine'  , pos = (0,17.916,-1))

        targetList = [self.aSpineTarg , self.bSpineTarg]
        self.spineTargGrp= Ut.jointTargViz(list=targetList, name='spine')



    @undoFunc
    def createGobalHeadTargets(self, *args):

        headPosQuery = ['jaw','headTop',' headRoot','headBot', 'neck_01', 'neck_02', 'L_eye', 'L_socket', 'R_eye','R_socket']
        if objExists(self.dataNode) and self.dataNode.hasAttr(headPosQuery[0]+'X') ==True :

            headPosVecs = self.getPosVecData(headPosQuery)

            print ('Global head TargetLocation Imported from metaData Node')
            self.jawTarg       =  Ut.jointTarget(name  = 'jaw'     , pos = (headPosVecs[0]))
            self.headTopTarg   =  Ut.jointTarget(name  = 'headTop' , pos = (headPosVecs[1]))
            self.headRootTarg  =  Ut.jointTarget(name  = 'headRoot', pos = (headPosVecs[2]))
            self.headBotTarg   =  Ut.jointTarget(name  = 'headBot' , pos = (headPosVecs[3]))
            self.neck01Targ    =  Ut.jointTarget(name  = 'neck_01' , pos = (headPosVecs[4]))
            self.neck02Targ    =  Ut.jointTarget(name  = 'neck_02' , pos = (headPosVecs[5]))
            self.lEyeTarg      =  Ut.jointTarget(name  = 'L_eye'   , pos = (headPosVecs[6]))
            self.lSocketTarg   =  Ut.jointTarget(name  = 'L_socket', pos = (headPosVecs[7]))
            self.rEyeTarg      =  Ut.jointTarget(name  = 'R_eye'   , pos = (headPosVecs[8]))
            self.rSocketTarg   =  Ut.jointTarget(name  = 'R_socket', pos = (headPosVecs[9]))
        else:
            self.jawTarg       =  Ut.jointTarget(name  = 'jaw'     , pos = (0,28.8,0))
            self.headTopTarg   =  Ut.jointTarget(name  = 'headTop' , pos = (0,32.6,.9))
            self.headRootTarg   =  Ut.jointTarget(name = 'headRoot', pos = (0,31.5,0))
            self.headBotTarg   =  Ut.jointTarget(name  = 'headBot' , pos = (0,32.7,.9))
            self.neck01Targ    =  Ut.jointTarget(name  = 'neck_01' , pos = (0,28.8,0))
            self.neck02Targ    =  Ut.jointTarget(name  = 'neck_02' , pos = (0,30.1,0))
            self.lEyeTarg      =  Ut.jointTarget(name  = 'L_eye'   , pos = (.68,33,0))
            self.lSocketTarg   =  Ut.jointTarget(name  = 'L_socket', pos = (.68,33,0))
            self.rEyeTarg      =  Ut.jointTarget(name  = 'R_eye'   , pos = (-.68,33,0))
            self.rSocketTarg   =  Ut.jointTarget(name  = 'R_socket', pos = (-.68,33,0))



    @undoFunc
    def createHandTargets(self, *args):

        handPosQuery = ['L_Thumb_01' ,'L_Thumb_02' ,'L_Thumb_03' ,'L_Thumb_04' ,
                        'L_index_01' ,'L_index_02' ,'L_index_03' ,'L_index_04' ,'L_index_05',
                        'L_Middle_01','L_Middle_02','L_Middle_03','L_Middle_04','L_Middle_05',
                        'L_Ring_01'  ,'L_Ring_02'  ,'L_Ring_03'  ,'L_Ring_04'  ,'L_Ring_05',
                        'L_Pinky_01' ,'L_Pinky_02' ,'L_Pinky_03' ,'L_Pinky_04' ,'L_Pinky_04']

        if objExists(self.dataNode) and self.dataNode.hasAttr(handPosQuery[0]+'X') ==True :

            handPosVecs = self.getPosVecData(handPosQuery)

            print ('Hand TargetLocation Imported from metaData Node')
            self.thumb01Targ  =  Ut.jointTarget(name  = 'Thumb_01'  , pos = (handPosVecs[0]))
            self.thumb02Targ  =  Ut.jointTarget(name  = 'Thumb_02'  , pos = (handPosVecs[1]))
            self.thumb03Targ  =  Ut.jointTarget(name  = 'Thumb_03'  , pos = (handPosVecs[2]))
            self.thumb04Targ  =  Ut.jointTarget(name  = 'Thumb_04'  , pos = (handPosVecs[3]))

            self.index01Targ  =  Ut.jointTarget(name  = 'index_01'  , pos = (handPosVecs[4]))
            self.index02Targ  =  Ut.jointTarget(name  = 'index_02'  , pos = (handPosVecs[5]))
            self.index03Targ  =  Ut.jointTarget(name  = 'index_03'  , pos = (handPosVecs[6]))
            self.index04Targ  =  Ut.jointTarget(name  = 'index_04'  , pos = (handPosVecs[7]))
            self.index05Targ  =  Ut.jointTarget(name  = 'index_05'  , pos = (handPosVecs[8]))

            self.middle01Targ =  Ut.jointTarget(name  = 'Middle_01' , pos = (handPosVecs[9]))
            self.middle02Targ =  Ut.jointTarget(name  = 'Middle_02' , pos = (handPosVecs[10]))
            self.middle03Targ =  Ut.jointTarget(name  = 'Middle_03' , pos = (handPosVecs[11]))
            self.middle04Targ =  Ut.jointTarget(name  = 'Middle_04' , pos = (handPosVecs[12]))
            self.middle05Targ =  Ut.jointTarget(name  = 'Middle_05' , pos = (handPosVecs[13]))

            self.ring01Targ   =  Ut.jointTarget(name  = 'Ring_01'   , pos = (handPosVecs[14]))
            self.ring02Targ   =  Ut.jointTarget(name  = 'Ring_02'   , pos = (handPosVecs[15]))
            self.ring03Targ   =  Ut.jointTarget(name  = 'Ring_03'   , pos = (handPosVecs[16]))
            self.ring04Targ   =  Ut.jointTarget(name  = 'Ring_04'   , pos = (handPosVecs[17]))
            self.ring05Targ   =  Ut.jointTarget(name  = 'Ring_05'   , pos = (handPosVecs[18]))

            self.pinky01Targ  =  Ut.jointTarget(name  = 'Pinky_01'  , pos = (handPosVecs[19]))
            self.pinky02Targ  =  Ut.jointTarget(name  = 'Pinky_02'  , pos = (handPosVecs[20]))
            self.pinky03Targ  =  Ut.jointTarget(name  = 'Pinky_03'  , pos = (handPosVecs[21]))
            self.pinky04Targ  =  Ut.jointTarget(name  = 'Pinky_04'  , pos = (handPosVecs[22]))
            self.pinky05Targ  =  Ut.jointTarget(name  = 'Pinky_05'  , pos = (handPosVecs[23]))

        else:
            self.thumb01Targ  =  Ut.jointTarget(name  = 'Thumb_01'  , pos = (9.666, 19.206, 1.236))
            self.thumb02Targ  =  Ut.jointTarget(name  = 'Thumb_02'  , pos = (9.612, 18.672, 2.004))
            self.thumb03Targ  =  Ut.jointTarget(name  = 'Thumb_03'  , pos = (9.678, 18.174, 2.514))
            self.thumb04Targ  =  Ut.jointTarget(name  = 'Thumb_04'  , pos = (9.75, 17.748, 2.814))


            self.index01Targ  =  Ut.jointTarget(name  = 'index_01'  , pos = (9.696, 19.572, 1.02))
            self.index02Targ  =  Ut.jointTarget(name  = 'index_02'  , pos = (10.89, 18.39, 1.95))
            self.index03Targ  =  Ut.jointTarget(name  = 'index_03'  , pos = (11.256, 17.766, 2.406))
            self.index04Targ  =  Ut.jointTarget(name  = 'index_04'  , pos = (11.562, 17.1, 2.73))
            self.index05Targ  =  Ut.jointTarget(name  = 'index_05'  , pos = (11.772, 16.554, 3.0))

            self.middle01Targ =  Ut.jointTarget(name  = 'Middle_01' , pos = (9.834, 19.536, 0.792))
            self.middle02Targ =  Ut.jointTarget(name  = 'Middle_02' , pos = (11.028, 18.198, 1.548))
            self.middle03Targ =  Ut.jointTarget(name  = 'Middle_03' , pos = (11.502, 17.652, 1.842))
            self.middle04Targ =  Ut.jointTarget(name  = 'Middle_04' , pos = (11.91, 16.95, 2.166))
            self.middle05Targ =  Ut.jointTarget(name  = 'Middle_05' , pos = (12.168, 16.308, 2.436))

            self.ring01Targ   =  Ut.jointTarget(name  = 'Ring_01'   , pos = (10.182, 19.368, 0.402))
            self.ring02Targ   =  Ut.jointTarget(name  = 'Ring_02'   , pos = (11.088, 18.066, 0.984))
            self.ring03Targ   =  Ut.jointTarget(name  = 'Ring_03'   , pos = (11.604, 17.484, 1.194))
            self.ring04Targ   =  Ut.jointTarget(name  = 'Ring_04'   , pos = (11.982, 16.854, 1.392))
            self.ring05Targ   =  Ut.jointTarget(name  = 'Ring_05'   , pos = (12.24, 16.344, 1.536))

            self.pinky01Targ  =  Ut.jointTarget(name  = 'Pinky_01'  , pos = (10.032, 19.05, 0.078))
            self.pinky02Targ  =  Ut.jointTarget(name  = 'Pinky_02'  , pos = (11.076, 17.976, 0.426))
            self.pinky03Targ  =  Ut.jointTarget(name  = 'Pinky_03'  , pos = (11.454, 17.496, 0.606))
            self.pinky04Targ  =  Ut.jointTarget(name  = 'Pinky_04'  , pos = (11.754, 16.962, 0.702))
            self.pinky05Targ  =  Ut.jointTarget(name  = 'Pinky_05'  , pos = (11.946, 16.584, 0.762))

        pinkyTarg = [self.pinky01Targ, self.pinky02Targ, self.pinky03Targ, self.pinky04Targ, self.pinky05Targ]
        ringTarg = [self. ring01Targ, self.ring02Targ, self.ring03Targ, self.ring04Targ, self.ring05Targ]
        middleTarg = [self.middle01Targ, self.middle02Targ, self.middle03Targ, self.middle04Targ, self.middle05Targ]
        indexTarg = [self.index01Targ, self.index02Targ, self.index03Targ, self.index04Targ, self.index05Targ]
        thumbTarg = [self.thumb01Targ, self.thumb02Targ, self.thumb03Targ, self.thumb04Targ]
        handTarg = [pinkyTarg,ringTarg, middleTarg,indexTarg, thumbTarg]

        self.handTargGrp=[]
        for i in handTarg:

            handTargGrp=  Ut.jointTargViz(name =  'hand' , list = i[::-1], r= .25)
            self.handTargGrp.append(handTargGrp)

        try:
            Ut.connectProtoHand(Driver = self.armTargGrp[-1],Driven = self.handTargGrp)
        except:
            pass

#___________Create___Bind____Joints________________________________________________________#
    @undoFunc
    def createLeftHand(self, *args):

        self.Thumb01Bnd  =  Ut.jntToTarg(  self.thumb01Targ  , prefix = 'L_'   ,  jntName = 'Thumb_01')
        self.Thumb02Bnd  =  Ut.jntToTarg(  self.thumb02Targ  , prefix = 'L_'   ,  jntName = 'Thumb_02')
        self.Thumb03Bnd  =  Ut.jntToTarg(  self.thumb03Targ  , prefix = 'L_'   ,  jntName = 'Thumb_03')
        self.Thumb04Bnd  =  Ut.jntToTarg(  self.thumb04Targ  , prefix = 'L_'   ,  jntName = 'Thumb_04')
        self.lThumbChain = [self.Thumb01Bnd, self.Thumb02Bnd, self.Thumb03Bnd, self.Thumb04Bnd]
        self.index01Bnd  =  Ut.jntToTarg(  self.index01Targ  , prefix = 'L_'   ,  jntName = 'index_01')
        self.index02Bnd  =  Ut.jntToTarg(  self.index02Targ  , prefix = 'L_'   ,  jntName = 'index_02')
        self.index03Bnd  =  Ut.jntToTarg(  self.index03Targ  , prefix = 'L_'   ,  jntName = 'index_03')
        self.index04Bnd  =  Ut.jntToTarg(  self.index04Targ  , prefix = 'L_'   ,  jntName = 'index_04')
        self.index05Bnd  =  Ut.jntToTarg(  self.index05Targ  , prefix = 'L_'   ,  jntName = 'index_05')
        self.lIndexChain = [self.index01Bnd, self.index02Bnd, self.index03Bnd, self.index04Bnd, self.index05Bnd]
        self.Middle01Bnd =  Ut.jntToTarg(  self.middle01Targ , prefix = 'L_'   ,  jntName = 'Middle_01')
        self.Middle02Bnd =  Ut.jntToTarg(  self.middle02Targ , prefix = 'L_'   ,  jntName = 'Middle_02')
        self.Middle03Bnd =  Ut.jntToTarg(  self.middle03Targ , prefix = 'L_'   ,  jntName = 'Middle_03')
        self.Middle04Bnd =  Ut.jntToTarg(  self.middle04Targ , prefix = 'L_'   ,  jntName = 'Middle_04')
        self.Middle05Bnd =  Ut.jntToTarg(  self.middle05Targ , prefix = 'L_'   ,  jntName = 'Middle_05')
        self.lMiddleChain= [self.Middle01Bnd, self.Middle02Bnd, self.Middle03Bnd, self.Middle04Bnd, self.Middle05Bnd]
        self.Ring01Bnd   =  Ut.jntToTarg(  self.ring01Targ   , prefix = 'L_'   ,  jntName = 'Ring_01')
        self.Ring02Bnd   =  Ut.jntToTarg(  self.ring02Targ   , prefix = 'L_'   ,  jntName = 'Ring_02')
        self.Ring03Bnd   =  Ut.jntToTarg(  self.ring03Targ   , prefix = 'L_'   ,  jntName = 'Ring_03')
        self.Ring04Bnd   =  Ut.jntToTarg(  self.ring04Targ   , prefix = 'L_'   ,  jntName = 'Ring_04')
        self.Ring05Bnd   =  Ut.jntToTarg(  self.ring05Targ   , prefix = 'L_'   ,  jntName = 'Ring_05')
        self.lRingChain  = [self.Ring01Bnd, self.Ring02Bnd, self.Ring03Bnd, self.Ring04Bnd, self.Ring05Bnd]
        self.pinky01Bnd  =  Ut.jntToTarg(  self.pinky01Targ  , prefix = 'L_'   ,  jntName = 'Pinky_01')
        self.pinky02Bnd  =  Ut.jntToTarg(  self.pinky02Targ  , prefix = 'L_'   ,  jntName = 'Pinky_02')
        self.pinky03Bnd  =  Ut.jntToTarg(  self.pinky03Targ  , prefix = 'L_'   ,  jntName = 'Pinky_03')
        self.pinky04Bnd  =  Ut.jntToTarg(  self.pinky04Targ  , prefix = 'L_'   ,  jntName = 'Pinky_04')
        self.pinky05Bnd  =  Ut.jntToTarg(  self.pinky05Targ  , prefix = 'L_'   ,  jntName = 'Pinky_05')
        self.lPinkyChain = [self.pinky01Bnd, self.pinky02Bnd, self.pinky03Bnd, self.pinky04Bnd, self.pinky05Bnd]
        self.lHandChain  = [self.lThumbChain,self.lIndexChain,self.lMiddleChain,self.lRingChain,self.lPinkyChain]

        for jntChain in self.lHandChain:
            self.populateData(jntChain)

        for jntChain in self.lHandChain:
            self.updateDataNode(jntChain)

        for jntChain in self.lHandChain:
            Ut.createHeiarchy(jntChain)

        for jntChain in self.lHandChain:
            Ut.orientJntChain(jntChain)

        self.rHandChain  = [Ut.mirrorChain(jntChain) for jntChain in self.lHandChain]
        self.lHandChildren    = [root[0] for root in self.lHandChain]
        self.rHandChildren    = [root[0] for root in self.rHandChain]
        lHandTFM         = Ut.groupHeiarchy(name = 'L_hand_BND_TFM', target =self.lArmChain[-1], deleteConstraint = False, children = self.lHandChildren)
        rHandTFM         = Ut.groupHeiarchy(name = 'R_hand_BND_TFM', target =self.rArmChain[-1], deleteConstraint = False, children = self.rHandChildren)

        for jntChain in self.rHandChain:
           Ut.orientJntChain(jntChain)
        delete(self.handTargGrp)
    @undoFunc
    def createLeftLeg(self, *args):

        '''create joints at target locations'''

        self.toeBnd      =  Ut.jntToTarg(  self.toeTarg       , prefix =  'L_',     jntName = 'toe')
        self.ballBnd     =  Ut.jntToTarg(  self.ballTarg      , prefix =  'L_',     jntName = 'ball')
        self.ankleBnd    =  Ut.jntToTarg(  self.ankleTarg     , prefix =  'L_',     jntName = 'ankle')
        self.kneeBnd     =  Ut.jntToTarg(  self.kneeTarg      , prefix =  'L_',     jntName = 'knee')
        self.hipBnd      =  Ut.jntToTarg(  self.hipTarg       , prefix =  'L_',     jntName = 'hip')

        self.lLegChain = [self.hipBnd ,self.kneeBnd ,self.ankleBnd ,self.ballBnd, self.toeBnd]
        self.populateData(self.lLegChain)
        self.updateDataNode(self.lLegChain)
        Ut.createHeiarchy(self.lLegChain)
        Ut.orientJntChain(self.lLegChain)
        self.rLegChain = Ut.mirrorChain(self.lLegChain)
        Ut.orientJntChain(self.rLegChain, mirror =True)
        delete(self.legTargGrp)

    @undoFunc
    def createLeftArmChain(self, *args):

        '''create joints at target locations'''

        self.handBnd      =  Ut.jntToTarg(  self.handTarg     , prefix =  'L_',     jntName = 'hand')
        self.wristBnd     =  Ut.jntToTarg(  self.wristTarg    , prefix =  'L_',     jntName = 'wrist')
        self.elbowBnd     =  Ut.jntToTarg(  self.elbowTarg    , prefix =  'L_',     jntName = 'elbow')
        self.shoulderBnd  =  Ut.jntToTarg(  self.shoulderTarg , prefix =  'L_',     jntName = 'shoulder')
        self.clavicleBnd  =  Ut.jntToTarg(  self.clavicleTarg , prefix =  'L_',     jntName = 'clavicle')


        self.lArmChain = [self.clavicleBnd ,self.shoulderBnd ,self.elbowBnd ,self.wristBnd ,self.handBnd]
        self.populateData(self.lArmChain)
        self.updateDataNode(self.lArmChain)
        Ut.createHeiarchy(self.lArmChain)
        Ut.orientJntChain(self.lArmChain)
        self.rArmChain = Ut.mirrorChain(self.lArmChain)
        Ut.orientJntChain(self.rArmChain,mirror= True)
        delete(self.armTargGrp)

    @undoFunc
    def createSpineChain(self, *args):

        '''create joints at target locations'''

        self.bSpineBnd     =  Ut.jntToTarg(  self.bSpineTarg    , jntName = 'bSpine')
        self.aSpineBnd     =  Ut.jntToTarg(  self.aSpineTarg    , jntName = 'aSpine')

        self.spineChain = [self.aSpineBnd ,self.bSpineBnd ]
        self.populateData(self.spineChain)
        self.updateDataNode(self.spineChain)
        Ut.createHeiarchy(self.spineChain)
        Ut.orientJntChain(self.spineChain)
        delete(self.spineTargGrp)

    @undoFunc
    def createGlobalHeadChain(self, *args):


        '''create joints at target locations'''

        self.jawBnd        =  Ut.jntToTarg(  self.jawTarg       , jntName = 'jaw')
        self.headTopBnd    =  Ut.jntToTarg(  self.headTopTarg   , jntName = 'headTop')
        self.headRootBnd   =  Ut.jntToTarg(  self.headRootTarg  , jntName = 'headRoot')
        self.headBotBnd    =  Ut.jntToTarg(  self.headBotTarg   , jntName = 'headBot')
        self.neck01Bnd     =  Ut.jntToTarg(  self.neck01Targ    , jntName = 'neck_01')
        self.neck02Bnd     =  Ut.jntToTarg(  self.neck02Targ    , jntName = 'neck_02')
        self.lEyeBnd       =  Ut.jntToTarg(  self.lEyeTarg      , jntName = 'L_eye')
        self.rEyeBnd       =  Ut.jntToTarg(  self.rEyeTarg      , jntName = 'R_eye')
        self.lSocketBnd    =  Ut.jntToTarg(  self.lSocketTarg   , jntName = 'L_socket')
        self.rSocketBnd    =  Ut.jntToTarg(  self.rSocketTarg   , jntName = 'R_socket')

        self.globalHeadChain = [ self.neck01Bnd, self.neck02Bnd, self.headRootBnd,  self.headBotBnd, self.jawBnd, self.headTopBnd, self.lSocketBnd, self.lEyeBnd,  self.rSocketBnd  , self.rEyeBnd]
        self.populateData(self.globalHeadChain)
        self.updateDataNode(self.globalHeadChain)

#___________Create___Driver____Joints________________________________________________________#

    @undoFunc
    def createArmDrvs(self, *args):

        '''creates the FK and IK drivers for the arm bind joints'''

        self.lArmFkChain             = Ut.createFkIkChain(self.lArmChain[1:4], grpName = 'L_FK_Arm',  delet =True, IK = False)
        self.lArmIkChain ,lIkHandle  = Ut.createFkIkChain(self.lArmChain[1:4], grpName = 'L_IK_Arm',  delet =True)
        self.rArmFkChain             = Ut.createFkIkChain(self.rArmChain[1:4], grpName = 'R_FK_Arm',  delet =True, IK = False)
        self.rArmIkChain ,rIkHandle  = Ut.createFkIkChain(self.rArmChain[1:4], grpName = 'R_IK_Arm',  delet =True)
        hide(self.rArmFkChain, self.lArmFkChain, self.rArmIkChain, self.lArmIkChain)
        lFkArmGrp = Ut.groupHeiarchy(name = 'L_FK_Arm_TFM', target = self.lArmChain[0], deleteConstraint = False, children = self.lArmFkChain[0])
        lIkArmGrp = Ut.groupHeiarchy(name = 'L_IK_Arm_TFM', target = self.lArmChain[0], deleteConstraint = False, children = self.lArmIkChain[0])
        rFkArmGrp = Ut.groupHeiarchy(name = 'R_FK_Arm_TFM', target = self.rArmChain[0], deleteConstraint = False, children = self.rArmFkChain[0])
        rIkArmGrp = Ut.groupHeiarchy(name = 'R_IK_Arm_TFM', target = self.rArmChain[0], deleteConstraint = False, children = self.rArmIkChain[0])

        ikHandles = [lIkHandle, rIkHandle]
        ikCtrls = Ut.createIkHandleCtrl(ikHandles = ikHandles, rotation = (0, 0, 90),r = 2, hr =.3, s =4, d=1)

        rArmPolePos = Ut.getPoleVectorPosition(self.rArmIkChain, 5, curveGuide = False)
        lArmPolePos = Ut.getPoleVectorPosition(self.lArmIkChain, 5, curveGuide = False)
        
        rArmPole = Ut.createPoleVector(name = 'R_Arm_PoleVector',ikHandle =ikHandles[-1] , poleVectorLocation = rArmPolePos, rotate = (-90,0,0),r = .65, hr =2)
        lArmPole = Ut.createPoleVector(name = 'L_Arm_PoleVector',ikHandle =ikHandles[0] , poleVectorLocation = lArmPolePos, rotate = (-90,0,0),r = .65, hr =2)

        rFkCtrls, lFkCtrls, fkCtrls = Ut.createCtrls(lFkChain = self.lArmFkChain , rFkChain =self.rArmFkChain, rotation = (0, 0, 90),hr = .07 ,radius = 2)
        
        self.rClavicleCtrls, self.lClavicleCtrls, clavCtrls  = Ut.createCtrls(type='cylinder', lFkChain = self.lArmChain[:1], 
                                                                            rFkChain = self.rArmChain[:1], rotation = (90, 0, 0), hr = 12 , r = .2)

        Ut.createCtrlHeiarchy(rFkCtrls)
        Ut.createCtrlHeiarchy(lFkCtrls)
        

        
        lIkCtrls, rIkCtrls = [ikCtrls[0],lArmPole],[ikCtrls[-1], rArmPole]
        
        Ut.attachToSubMaster(lIkCtrls+ rIkCtrls, self.subMasterCtrl)
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'L_Arm_IKFK_Switch', driven = lIkCtrls , drivenAttr ='visibility')
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'R_Arm_IKFK_Switch', driven = rIkCtrls , drivenAttr ='visibility')

        
        
        lFkCtrlTfm = [i[-1] for i in lFkCtrls]
        rFkCtrlTfm = [i[-1] for i in rFkCtrls]
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'L_Arm_IKFK_Switch', driven = lFkCtrlTfm , drivenAttr ='visibility', reverse =True )
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'R_Arm_IKFK_Switch', driven = rFkCtrlTfm , drivenAttr ='visibility', reverse =True )

        Ut.groupHeiarchy(name = 'L_ArmFk_Ctrl_GRP', target =self.lArmChain[0], deleteConstraint = False, children = lFkCtrls[0][0])
        Ut.groupHeiarchy(name = 'R_ArmFk_Ctrl_GRP', target =self.rArmChain[0], deleteConstraint = False, children = rFkCtrls[0][0])


        rArmBlndNames  = ['R_shoulder','R_elbow','R_wrist']
        self.rArmBlendNodes = Ut.blendJoints(self.rArmIkChain, self.rArmFkChain , self.rArmChain[1:4] , rArmBlndNames,'R_Arm_IKFK_Switch')
        lArmBlndNames  = ['L_shoulder','L_elbow','L_wrist']
        self.lArmBlendNodes = Ut.blendJoints(self.lArmIkChain, self.lArmFkChain , self.lArmChain[1:4] , lArmBlndNames,'L_Arm_IKFK_Switch')


        leftForearmRibbon,  lforearmMdl = Ut.createRibbon(name='L_Forearm', startLocation= self.lArmChain[2],
                                         endLocation= self.lArmChain[3], numberOfJoints = 5)
        leftBicepRibbon,    lBicepMdl   = Ut.createRibbon(name='L_Bicep', startLocation= self.lArmChain[1],
                                         endLocation= self.lArmChain[2], numberOfJoints = 5)
        rightBicepRibbon,   rBicepMdl   = Ut.createRibbon(name='R_Bicep', startLocation= self.rArmChain[1],
                                         endLocation= self.rArmChain[2], numberOfJoints = 5)
        rightForearmRibbon, rForeArmMdl = Ut.createRibbon(name='R_Forearm', startLocation= self.rArmChain[2],
                                         endLocation= self.rArmChain[3], numberOfJoints = 5)

        lBendyJnts =  leftBicepRibbon, leftForearmRibbon
        rBendyJnts =  rightBicepRibbon,rightForearmRibbon
        armMdls    =  lBicepMdl, lforearmMdl, rBicepMdl, rForeArmMdl
        

        rbendyCtrls, lbendyCtrls, bendyCtrls = Ut.createCtrls(type='cylinder', lFkChain = lBendyJnts, rFkChain = rBendyJnts, rotation = (0, 0, 90), hr = .3 , r = 1.7)
        Ut.createbendyCtrlHeiarchy(self.lArmChain[1:]+self.rArmChain[1:], bendyCtrls)
        
        Ut.connectBendyMdls(Mdls = armMdls, Ctrls = bendyCtrls)

        self.ctrlCleanList.append([fkCtrls,ikCtrls])
        try:
            lHandCtrlTFM         = Ut.groupHeiarchy(name = 'L_Arm_TFM', target =self.spineDrvJnts[-1], deleteConstraint = False, children =self.lClavicleCtrls[0][0])
            rHandCtrlTFM         = Ut.groupHeiarchy(name = 'R_Arm_TFM', target =self.spineDrvJnts[-1], deleteConstraint = False, children =self.rClavicleCtrls[0][0])
        except:
            pass

    @undoFunc
    def createLegDrvs(self, *args):

        self.lLegFkChain            = Ut.createFkIkChain(self.lLegChain, grpName = 'L_FK_Leg',  delet =False, IK = False)
        self.lLegIkChain, lIkHandle = Ut.createFkIkChain(self.lLegChain, grpName = 'L_IK_Leg',  delet =False)
        self.rLegFkChain            = Ut.createFkIkChain(self.rLegChain, grpName = 'R_FK_Leg',  delet =False, IK = False)
        self.rLegIkChain, rIkHandle = Ut.createFkIkChain(self.rLegChain, grpName = 'R_IK_Leg',  delet =False)
        self.rScIks                 = Ut.createSingleChainIks(self.rLegIkChain[-3:])
        self.lScIks                 = Ut.createSingleChainIks(self.lLegIkChain[-3:])
        hide(self.rLegFkChain, self.lLegFkChain, self.rLegIkChain, self.lLegIkChain)
        self.legIkHandles = [lIkHandle,rIkHandle]
        ikCtrls  = Ut.createIkHandleCtrl(type = 'sphere' ,ikHandles=self.legIkHandles,r = 1.3)
       

        rLegPolePos = Ut.getPoleVectorPosition(self.rLegIkChain[:3], 6, curveGuide=False)
        lLegPolePos =  Ut.getPoleVectorPosition(self.lLegIkChain[:3],6, curveGuide=False)
        
        
        rLegPole = Ut.createPoleVector(name = 'R_Leg_PoleVector',ikHandle =self.legIkHandles[-1] , poleVectorLocation = rLegPolePos, rotate = (90,0,0),r = .65, hr =2)
        lLegPole = Ut.createPoleVector(name = 'L_Leg_PoleVector',ikHandle =self.legIkHandles[0] , poleVectorLocation = lLegPolePos, rotate = (90,0,0),r = .65, hr =2)
        rLegPole.getParent().getParent().setParent(self.subMasterCtrl)
        lLegPole.getParent().getParent().setParent(self.subMasterCtrl)
        
        
        lIkCtrls, rIkCtrls = [ikCtrls[0],lLegPole],[ikCtrls[-1], rLegPole]
        
        Ut.attachToSubMaster(lIkCtrls+ rIkCtrls, self.subMasterCtrl)
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'L_Leg_IKFK_Switch', driven = lIkCtrls , drivenAttr ='visibility')
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'R_Leg_IKFK_Switch', driven = rIkCtrls , drivenAttr ='visibility')


        self.rLegFkCtrls, self.lLegFkCtrls, fkCtrls = Ut.createCtrls(lFkChain=self.lLegFkChain, rFkChain=self.rLegFkChain, rotation = (0, 0, 90),hr = .07 ,radius = 2)

        Ut.createCtrlHeiarchy(self.rLegFkCtrls)
        Ut.createCtrlHeiarchy(self.lLegFkCtrls)
        
        lFkCtrlTfm = [i[-1] for i in self.lLegFkCtrls]
        rFkCtrlTfm = [i[-1] for i in self.rLegFkCtrls]
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'L_Leg_IKFK_Switch', driven = lFkCtrlTfm , drivenAttr ='visibility', reverse =True )
        Ut.connectAttrs(driver= self.masterCtrl,driverAttr = 'R_Leg_IKFK_Switch', driven = rFkCtrlTfm , drivenAttr ='visibility', reverse =True )


        rArmBlndNames  = ['R_hip','R_knee','R_ankle','R_ball','R_toe']
        self.rLegBlendNodes = Ut.blendJoints(self.rLegIkChain, self.rLegFkChain , self.rLegChain , rArmBlndNames,'R_Leg_IKFK_Switch')
        lArmBlndNames  = ['L_hip','L_knee','L_ankle','L_ball','L_toe']
        self.lLegBlendNodes = Ut.blendJoints(self.lLegIkChain, self.lLegFkChain , self.lLegChain , lArmBlndNames,'L_Leg_IKFK_Switch')


        leftShinRibbon,   lShinMdl  = Ut.createRibbon(name='L_Shin', startLocation= self.lLegChain[1],
                                         endLocation= self.lLegChain[2], numberOfJoints = 5)
        leftThighRibbon,  lThighMdl = Ut.createRibbon(name='L_Thigh', startLocation= self.lLegChain[0],
                                         endLocation= self.lLegChain[1], numberOfJoints = 5)
        rightThighRibbon, rThighMdl = Ut.createRibbon(name='R_Thigh', startLocation= self.rLegChain[0],
                                         endLocation= self.rLegChain[1], numberOfJoints = 5)
        rightShinRibbon,  rShinMdl  = Ut.createRibbon(name='R_Shin', startLocation= self.rLegChain[1],
                                         endLocation= self.rLegChain[2], numberOfJoints = 5)

        lBendyJnts =  leftThighRibbon, leftShinRibbon
        rBendyJnts =  rightThighRibbon,rightShinRibbon
        legMdls    =  lThighMdl, lShinMdl, rThighMdl, rShinMdl
        
        rbendyCtrls, lbendyCtrls, bendyCtrls= Ut.createCtrls(type='cylinder', lFkChain = lBendyJnts, rFkChain = rBendyJnts, rotation = (0, 0, 90), hr = .3 , r = 1.7)
        Ut.createbendyCtrlHeiarchy(self.lLegChain+self.rLegChain, bendyCtrls)

        Ut.connectBendyMdls(Mdls = legMdls, Ctrls = bendyCtrls)
        
        Ut.createFootDrv(scIks = self.lScIks, rpIks = self.legIkHandles[0])
        Ut.createFootDrv(scIks = self.rScIks, rpIks = self.legIkHandles[-1])
        
        self.ctrlCleanList.append([fkCtrls, ikCtrls])

        try:
            lFkBndLegGrp = Ut.groupHeiarchy(name = 'L_Fk_Bnd_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegChain[0])
            rFkBndLegGrp = Ut.groupHeiarchy(name = 'R_Fk_Bnd_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegChain[0])

            lFkDrvLegGrp = Ut.groupHeiarchy(name = 'L_Fk_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegFkChain[0])
            lIkDrvLegGrp = Ut.groupHeiarchy(name = 'L_Ik_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegIkChain[0])
            rFkDrvLegGrp = Ut.groupHeiarchy(name = 'R_Fk_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegFkChain[0])
            rIkDrvLegGrp = Ut.groupHeiarchy(name = 'R_Ik_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegIkChain[0])


            lFkLegCTRLGrp = Ut.groupHeiarchy(name = 'L_Fk_Leg_CTRL_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegFkCtrls[0][0])
            rFkLegCTRLGrp = Ut.groupHeiarchy(name = 'R_Fk_Leg_CTRL_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegFkCtrls[0][0])
        except:
            pass
  
        
    @undoFunc
    def createHandDrvs(self):


        rKnuckles = [i[1] for i in self.rHandChain]
        lKnuckles = [i[1] for i in self.lHandChain]

        rKnuckleCtrls, lKnuckleCtrls, allKnuckleCtrls = Ut.createCtrls(type='cylinder', lFkChain= lKnuckles, rFkChain= rKnuckles, rotation=(0, 0, 0),hr = 12 , r = .2)

        iter=  0
        for i in zip(self.lHandChain , self.rHandChain):

            rhandCtrlChain, lhandCtrlChain, handCtrlChain = Ut.createCtrls(type='sphere', lFkChain=i[0][1:-1], rFkChain=i[1][1:-1], rotation=(0, 0, 0), r=.3)

            lhandCtrlChain.insert(0,lKnuckleCtrls[iter])
            rhandCtrlChain.insert(0,rKnuckleCtrls[iter])

            Ut.createCtrlHeiarchy(lhandCtrlChain)
            Ut.createCtrlHeiarchy(rhandCtrlChain)
            Ut.setHandDrivenKeys(rhandCtrlChain)
            Ut.setHandDrivenKeys(lhandCtrlChain)
            iter+=1

        lHandCtrls = [i.getParent().getParent() for i in allKnuckleCtrls if i.startswith("L_")]
        rHandCtrls = [i.getParent().getParent() for i in allKnuckleCtrls if i.startswith("R_")]

        if objExists(self.lArmChain[-1]) and objExists(self.rArmChain[-1]):
            lHandCtrlTFM         = Ut.groupHeiarchy(name = 'L_hand_CTRL_TFM', target =self.lArmChain[-1], deleteConstraint = False, children =lHandCtrls)
            rHandCtrlTFM         = Ut.groupHeiarchy(name = 'R_hand_CTRL_TFM', target =self.rArmChain[-1], deleteConstraint = False, children =rHandCtrls)
            

    @undoFunc
    def createSpineDrvs(self):
        ctrls, self.spineDrvJnts, root = Ut.createRibbon(name='Spine', startLocation=self.spineChain[0],
                                                  endLocation=self.spineChain[-1], numberOfJoints=10, type = 'spine',
                                                  s=4, ssw = 45 , esw =405, hr = .1 , r=3.8, d =1)
        pelvis    = ctrls[0][0][0]
        pelvis.setParent(root)
        root.getParent().getParent().setParent(self.subMasterCtrl)
        select(d=True)

        try:
            lHandCtrlTFM = Ut.groupHeiarchy(name = 'L_Arm_TFM', target =self.spineDrvJnts[-1], deleteConstraint = False, children =self.lClavicleCtrls[0][0])
            rHandCtrlTFM = Ut.groupHeiarchy(name = 'R_Arm_TFM', target =self.spineDrvJnts[-1], deleteConstraint = False, children =self.rClavicleCtrls[0][0])
        except:
            pass

        try:
            lFkBndLegGrp = Ut.groupHeiarchy(name = 'L_Fk_Bnd_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegChain[0])
            rFkBndLegGrp = Ut.groupHeiarchy(name = 'R_Fk_Bnd_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegChain[0])

            lFkDrvLegGrp = Ut.groupHeiarchy(name = 'L_Fk_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegFkChain[0])
            lIkDrvLegGrp = Ut.groupHeiarchy(name = 'L_Ik_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegIkChain[0])
            rFkDrvLegGrp = Ut.groupHeiarchy(name = 'R_Fk_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegFkChain[0])
            rIkDrvLegGrp = Ut.groupHeiarchy(name = 'R_Ik_Drv_Leg_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegIkChain[0])


            lFkLegCTRLGrp = Ut.groupHeiarchy(name = 'L_Fk_Leg_CTRL_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.lLegFkCtrls[0][0])
            rFkLegCTRLGrp = Ut.groupHeiarchy(name = 'R_Fk_Leg_CTRL_Grp', target = self.spineDrvJnts[0], deleteConstraint = False, children = self.rLegFkCtrls[0][0])
        except:
            pass

    @undoFunc
    def createHeadDrvs(self):
        
        neckCtrls = Ut.createCtrls(type = 'cylinder', lFkChain = self.globalHeadChain[:6], constraint = False, r = 1, hr= .3 , ssw = 180)
        Ut.createCtrlHeiarchy(neckCtrls[0][:-1])
        eyeCtrls = Ut.createCtrls(type = 'cylinder', lFkChain = self.globalHeadChain[-4:-2], rFkChain = self.globalHeadChain[-2:] , constraint = False, r = 1, hr= .3 , ssw = 180)
        Ut.createCtrlHeiarchy(eyeCtrls[0])
        Ut.createCtrlHeiarchy(eyeCtrls[1])
        rSocket = eyeCtrls[0][0][0]
        lSocket = eyeCtrls[1][0][0]
        headTop = neckCtrls[0][-1][-1]
        headTopTFM = neckCtrls[0][-1][0]
        headRoot= neckCtrls[0][2][-1]
        rSocket.setParent(headTop)
        lSocket.setParent(headTop)
        headTopTFM.setParent(headRoot)
        
       
if __name__ == "__main__":
    win = PxAutoRig()
    win.createArmTargets()
    win.createLegTargets()
    win.createSpineTargets()
    win.createGobalHeadTargets()
    win.createHandTargets()
    win.createLeftArmChain()
    win.createLeftLeg()
    win.createSpineChain()
    win.createGlobalHeadChain()
    win.createLeftHand()
    win.createArmDrvs()
    win.createLegDrvs()
    win.createSpineDrvs()
    win.createHandDrvs()
    win.createHeadDrvs()
   
    data =  PyNode('MetaData')
    #lockNode(data , l=False)
   

