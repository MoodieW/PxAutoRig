from pymel.core import *
import PxAutorig.Ctrls as Ct

reload(Ct)


def jointTarget(name='name', pos=(0, 0, 0)):
    '''create targets for joint placement'''

    select(d=True)
    target = joint(n=name + '_Targ', rad=.3)
    target.setTranslation(pos)
    select(d=True)
    return target

def addMembersToLayer(layer ='', member = None):
    """
    Creates a Layer if it does not exist and add members to it
    :param layer: name of the layer
    :param member: members you wish to add, can be string, variable, or list
    """
    if objExists(layer):
        jntLyr = PyNode(layer)
    else:
        jntLyr = createDisplayLayer(n = layer)

    if type(member) == list:
        for i in member:
            jntLyr.addMembers(i)
    else:
        jntLyr.addMembers(member)


def jntToTarg(target=None, prefix='', jntName='name'):
    '''snaps joints to targets and deletes the targets'''

    if objExists(target):
        select(d=True)
        jntName = prefix + jntName + '_BND'
        bndJnt = joint(name=jntName, rad=.5)
        constr = parentConstraint(target, bndJnt)
        delete(constr)
        delete(target)
        select(d=True)
        addMembersToLayer(layer = 'Joint_layer', member = bndJnt)
    else:
        raise RuntimeError('Target Chain Does Not Exists Please Create Targets....................."Dummy"')
    return bndJnt


def createHeiarchy(jntChain):
    '''
    createHeiarchy creates a simple heiarchy
    :param jntChain: pass a jnt list from root to tip I.E. [L_calvicle, L_shoulder,L_wrist...]
    '''

    count = 1
    i = -2
    for jnt in jntChain[::-1]:
        if i > len(jntChain) * -1 - 1:
            parent(jnt, jntChain[i])
            i -= 1
        else:
            pass


def createCtrlHeiarchy(ctrlList):
    '''
    creates a Heiarchy for ctrls.
    :param CtrlList: the ctrl list needs a list of tuples. The list consist of a Group holding the ctrl and the ctrl I.e.
                   [(nt.Transform(u'R_FK_shoulder_DRV_CTRL_TFM'), nt.Transform(u'R_FK_shoulder_DRV_ctrl')),
                    (nt.Transform(u'R_FK_elbow_DRV_CTRL_TFM')   , nt.Transform(u'R_FK_elbow_DRV_ctrl')),
                    (nt.Transform(u'R_FK_wrist_DRV_CTRL_TFM')   , nt.Transform(u'R_FK_wrist_DRV_ctrl'))]
    '''

    count = 1
    i = -2
    for ctrl in ctrlList[::-1]:
        if i > len(ctrlList) * -1 - 1:
            parent(ctrl[0], ctrlList[i][1])
            i -= 1
        else:
            pass
    select(d=True)


def createbendyCtrlHeiarchy(bendyJoints, CtrlList):
    '''
    orginizes the bendy joints into a heiarchy a
    :param bendyJoints: pass in a list with both the left and right chains. I.E [l_shoulder,l_elbow,l_wrist,]+[R_shoulder,R_elbow,R_wrist,]
    :param CtrlList: pass in left and right ctrls for the bendy joints:
    :return: The heiarchy groups to use for later.
    '''
    lBendyConstraints = [i for i in bendyJoints if i.startswith("L_")]
    rBendyConstraints = [i for i in bendyJoints if i.startswith("R_")]

    lBendy = [i.getParent().getParent() for i in CtrlList if i.startswith("L_")]
    lGrp = [group(i, n=i[:-12] + 'Ctrls_Grp') for i in lBendy]
    select(d=True)

    rBendy = [i.getParent().getParent() for i in CtrlList if i.startswith("R_")]
    rGrp = [group(i, n=i[:-12] + 'Ctrls_Grp') for i in rBendy]
    select(d=True)

    iteration = 0
    for i in lBendy:
        pointConstraint(lBendyConstraints[iteration], lBendyConstraints[iteration + 1], i, mo=True)
        orientConstraint(lBendyConstraints[iteration], i, mo=True)
        iteration += 1

    iteration = 0
    for i in rBendy:
        pointConstraint(rBendyConstraints[iteration], rBendyConstraints[iteration + 1], i, mo=True)
        orientConstraint(rBendyConstraints[iteration], i, mo=True)
        iteration += 1

    return lGrp, rGrp


def createCtrls(type='torus', lFkChain=[], rFkChain=[], rotation=(0, 0, 0), constraint=True ,**kwargs):
    '''
    creates Fk controls for two supplied chains
    :param lFkChain  : supply left FK Chain from root to tip
    :param rFkChain  : supply Right FK Chain from root to tip
    :param constraint: if constraint == True parent constarints will be used other wise the object will be parented directly
    :param **kwargs  : supply standard build arguments for nurbs geo I.E cmds.cylinder(**kwargs)
    :return: first two returns return a list of tuples to be used by the createCtrlHeiarchy function
            the third is to be appended to the clean ctrl list in in the auto rig class.
    '''
    type = type.lower()
    if lFkChain == [] and rFkChain == []:
        print ('please supply arguments')
    else:
        rFkCtrls = []
        lFkCtrls = []
        FkCtrls = []
        for jnts in lFkChain + rFkChain:
            if type == 'torus':
                ctrl = Ct.torusCtrl(name=jnts, **kwargs)
            elif type == 'sphere':
                ctrl = Ct.sphereCtrl(name=jnts, **kwargs)
            elif type == 'cone':
                ctrl = Ct.coneCtrl(name=jnts, **kwargs)
            else:
                ctrl = Ct.cylinderCtrl(name=jnts, **kwargs)

            ctrl.setRotation(rotate=rotation)
            if constraint == True:
                ctrl.setDriven(jnts)
            else:
                ctrl.setDriven(jnts, child=True)
            if jnts.startswith('L_'):
                lFkCtrls.append((ctrl.ctrlOffset, ctrl.ctrl[0]))
            else:
                rFkCtrls.append((ctrl.ctrlOffset, ctrl.ctrl[0]))
            FkCtrls.append(ctrl.ctrl[0])
            select(d=True)
        addMembersToLayer(layer = 'Control_Layer', member= FkCtrls)
        return rFkCtrls, lFkCtrls, FkCtrls


def createIkHandleCtrl(type = 'cylinder',ikHandles=None, rotation=(0, 0, 0), **kwargs):
    '''
    Creates controls on for your ik Handles.
    :param ikHandle: pass a list of ik Handles.
    :param rotation: If you want to rotate you controller to  make more visually match appealing.
    :return: Returns a list of controls to be cleaned later
    '''
    ctrls = []
    for i in ikHandles:
        if type == 'torus':
            ctrl = Ct.torusCtrl(name=i, **kwargs)
        elif type == 'sphere':
            ctrl = Ct.sphereCtrl(name=i, **kwargs)
        elif type == 'cone':
            ctrl = Ct.coneCtrl(name=i, **kwargs)
        else:
            ctrl = Ct.cylinderCtrl(name=i, **kwargs)
        ctrl.setRotation(rotate=rotation)
        ctrl.setChildIk(ikHandle=i)
        endJnt =  i.getJointList()[-1].getChildren()[0]
        orientConstraint(ctrl.ctrl[0] , endJnt,  mo=True)
        ctrls.append(ctrl.ctrl[0])
    addMembersToLayer(layer = 'Control_Layer', member= ctrls)
    return ctrls[:]

def orientJntChain(jntChain=[], mirror=False):
    '''
    orientJntChain will do a basic joint orientation and mirror if asked.
    :param JntChain: pass a list of joints that are already in a heaiarchy i.E.
    :param mirror: will mirror the orientation if mirror is set to True
    '''

    select(d=True)
    frontOfChain, endOfChain = jntChain[0], jntChain[-1]
    select(frontOfChain)
    joint(e=True, zso=True, oj='xzy', secondaryAxisOrient='zup', ch=True)
    select(d=True)
    endOfChain.setOrientation([0.0, 0.0, 0.0, 1.0])
    select(d=True)
    if mirror == True:
        for i in jntChain:
            parent(i, world=True)
        for i in jntChain:
            i.rotate.set(180, 0, 180)
            makeIdentity(i, a=True, rotate=True)
        createHeiarchy(jntChain)



def mirrorChain(jntChain):
    '''
    mirror Chain will do a basic joint orientation and mirror if asked.
    :param JntChain: pass a list of joints that are already in a heaiarchy i.E.
    :param mirror: will mirror the orientation for mirrored actions.
    :return: will return a list of pyNodes for the newely created joint chain.
    '''
    select(d=True)
    chainRoot = jntChain[0]
    select(chainRoot)
    mirrorJoint(mirrorYZ=True, mirrorBehavior=True, searchReplace=('L_', 'R_'))
    select(hi=True)
    mirrorChain = [PyNode(i) for i in ls(sl=True, type='joint')]
    select(d=True)
    return mirrorChain


def lockTranslate(object, lockAttr='xyz'):
    '''
    lock and hide Translates.
    :param lockAttr: specfiy the axis you want to disable in a single string.
    '''

    for axis in lockAttr:
    
        setAttr(object + '.t' + axis, k=False, l=True)


def lockScale(object, lockAttr='xyz'):
    '''
    lock and hide Scale.
    :param lockAttr: specfiy the axis you want to disable in a single string.
    '''

    for axis in lockAttr:
        setAttr(object + '.s' + axis, k=False, l=True)


def lockRotate(object, lockAttr='xyz'):
    '''
    lock and hide Rotate.
    :param lockAttr: specfiy the axis you want to disable in a single string.
    '''

    for axis in lockAttr:
        setAttr(object + '.r' + axis, k=False, l=True)


def matchTrans(driver, driven, cn=True, mo=False):
    '''
    match the Translation of a specified driver.
    :param driver: supply the driver object.
    :param driven: supply the driven object
    :param cn: delete the constraint if cn == True
    :param mo: maintain offset if mo == True
    '''
    if cn == True:
        delete(parentConstraint(driver, driven, mo=mo))
    else:
        parentConstraint(driver, driven, mo=False)


def groupHeiarchy(name='', target=None, deleteConstraint=True, children=None, maintainOffset=False):
    '''
    groupHeiarchy groups and object and constrains that group toa driver .
    :param driver: supply the driver object.
    :param driven: supply the driven object
    :param cn: delete the constraint if cn == True
    :param mo: maintain offset if mo == True
    '''
    if target == None:
        print 'target give to arguments'
    else:
        grp = group(n=name)
        select(d=True)
        if maintainOffset == False:
            matchTrans(target, grp, cn=deleteConstraint)
        else:
            matchTrans(target, grp, cn=deleteConstraint, mo=maintainOffset)
        select(d=True)
        parent(children, grp)
        select(d=True)
        return grp


def lockViz(object):
    '''
    lock and hide visibilty attr
    :param: pass the object you want to hade the visibilty attr on.
    '''
    setAttr(object + '.v', k=False, l=True)


def createFkIkChain(jntChain, delet=False, grpName='', IK=True):
    ''' 
    Creates an fk or Ik Chain from your bind skeleton
    :param jntChain: pass a joint Chain from root to tip I.E [L_calvicle, L_shoulder,L_wrist...]
    :param delet: Mark as true if you want to delete the everything after the 3rd joint in the the list
    :grpName : pass a string name of the chain group
    :IK : Mark as False if you want to make Fk chain
    :return: returns joint list and ik handles
    '''

    fk = duplicate(jntChain[0], rc=True)
    parent(select(fk), w=True)
    temp = fk[0].listRelatives(ad=True)
    temp = temp[::-1]
    temp.insert(0, fk[0])
    if IK == True:
        jointList = [rename(jnt, jnt[:2] + 'IK_' + jnt[2:-5] + '_DRV') for jnt in temp]
    else:
        jointList = [rename(jnt, jnt[:2] + 'FK_' + jnt[2:-5] + '_DRV') for jnt in temp]
    select(jointList[0])
    count = 1
    if delet == False:
        pass
    else:
        while count != 4:
            pickWalk(d='down')
            count += 1
        delete()
        del jointList[3:]
    if IK == True:
        ik = ikHandle(n=grpName + '_IkHandle', sol='ikRPsolver', sj=temp[0], ee=temp[2])
    select(d=True)
    
    if IK == False:
        return jointList
    else:
        return jointList, ik[0]

def createSingleChainIks(jointList):
    '''
    Creates single chain ik solvers for a list of joints
    :param jointList: pass a joint Chain from root to tip I.E [L_Ankle, L_BallFoot,L_Toe...]
    :return: returns ik handles
    '''
    
    iks=[]
    iter = 1
    for i in jointList:
        if iter == len(jointList):
            pass
        else:
            ik = ikHandle(n=jointList[iter] + '_IkHandle', sol='ikSCsolver', sj=i, ee=jointList[iter])
            iter+=1
            iks.append(ik[0])
            select(d=True)
    return iks
        
def blendJoints(ikJoints, fkJoints, skinJoints, name, attr):
    '''
    Creates blend color nodes for ik fk switching
    :param ikJoints: pass a joint Chain from root to tip I.E [L_Ankle, L_BallFoot,L_Toe...]
    :param FkJoints: pass a joint Chain from root to tip I.E [L_Ankle, L_BallFoot,L_Toe...]
    :param skinJoints: pass a joint Chain from root to tip I.E [L_Ankle, L_BallFoot,L_Toe...]
    :param name: pass a list of names I.E [L_Ankle, L_BallFoot,L_Toe...]
    :param Attr: pass a string that be hooked into the master controls  I.E 'R_Arm_IKFK_Switch'
    :return: returns the blend node list
    '''
    drivers = zip(ikJoints, fkJoints, skinJoints, name)
    blndList = []
    masterCtrl = PyNode('Master_CTRL')
    for jnts in drivers:
        # order is IK - FK - Main
        blndRot = shadingNode('blendColors', asUtility=True, n='IKFK_Rotate_' + jnts[-1] + '_Blnd')
        masterCtrl.attr(attr) >> blndRot.blender
        jnts[0].rotate >> blndRot.color1
        jnts[1].rotate >> blndRot.color2
        blndRot.output >> jnts[2].rotate


        blndTFM = shadingNode('blendColors', asUtility=True, n='IKFK_Translate_' + jnts[-1] + '_Blnd')
        masterCtrl.attr(attr) >> blndTFM.blender
        jnts[0].translate >> blndTFM.color1
        jnts[1].translate >> blndTFM.color2
        blndTFM.output >> jnts[2].translate
        blndList.append(blndRot)
        blndList.append(blndTFM)

    return blndList

def connectBendyMdls(Mdls = None, Ctrls = None):
    '''
    connect Bendy Mdls
    :param Mdls: pass a list of a mult double linear nodes
    :param Ctrls: pass a list of the bendy Ctrls
    '''
    temp = zip(Mdls, Ctrls)
    for i in temp:
        Mdl = i[0]
        ctrlLoc = i[-1].getParent()
        
        Mdl.output >> ctrlLoc.rotateX
    
def createRibbon(name='', startLocation=None, endLocation=None, numberOfJoints=3, type='limb', **kwargs):
    '''
    Creates Ribbon
    :param name: pass a name string
    :param startLocation: pass a start location for the ribbon
    :param EndLocation: pass a end location for the ribbon
    :param Number Of Joints: pass an interger that will determine how joints will attach the the ribbon
    :param type: pass the string 'spine' if you want to make a spine ribbon other wise leave it default
    :param **kwargs : pass some argument for a spine control creation  if type == 'spine'
    :return: returns ctrls, drvJntsList, folGrp else return folGrp
    '''

    def spineRibbon():
        '''
        takes the ribbon and creates a spine variant of a ribbon
        :return:
        '''
        ribbon[0].rotate.set((0, 90, 0))
        makeIdentity(ribbon[0], a=True, r=True, pn=True)
        select(d=True)
        tailVec = startLocation.getTranslation(space='world')
        posVec = endLocation.getTranslation(space='world') - startLocation.getTranslation(space='world')
        numCtrls = 4
        numCtrls -= 1
        ctrlRatio = posVec / numCtrls

        i = 0
        drvJntsList = []
        while i <= numCtrls:
            # creates driver joints and places them
            drvJnt = joint(n=name + '_' + str(i) + '_FOL_DRV', rad=.25)
            ctrlLocation = tailVec + ctrlRatio * i
            xform(t=(ctrlLocation))
            select(cl=True)
            drvJntsList.append(drvJnt)
            i += 1
        addMembersToLayer(layer='Joint_layer', member=drvJntsList)
        ctrls = createCtrls(type='cylinder  ', lFkChain=drvJntsList, constraint=False, **kwargs)
        createCtrlHeiarchy(ctrls[0])
        delete(startLocation)
        select(drvJntsList[:], ribbon[0])
        skinCluster(tsb=True)
        select(d=True)
        select(ctrls[0][0][0], folGrp, ribbon[0])
        group(n=name + '_GRP')
        select(d=True)
        root = Ct.cylinderCtrl(name = 'Root_', r = 4.5, ssw = 45, esw = 405, d = 1, sections = 4, hr= 0.1) 
        root.setPosition(parent = drvJntsList[0])
        ctrls[0][1][0].setParent(root.ctrl[0])
        addMembersToLayer(layer = 'Control_Layer', member= root.ctrl[0])
        return ctrls, drvJntsList, root.ctrl[0]

    def limbRibbon():
        endAnchor = duplicate(endLocation, n=startLocation[:-4] + '_Anchor')
        delete(listRelatives(endAnchor[0], c=True))
        aimConstraint(startLocation, endAnchor,  aim=[-1, 0, 0], wut='objectRotation', wuo= endLocation)
        midJnt = joint(n=name + '_Bend_Bnd')
        grp = group(midJnt, n=midJnt[:-4] + '_TFM')
        delete(parentConstraint(startLocation, grp, mo=False))
        grp.translate.set(midPos)
        pointConstraint(startLocation, endAnchor, grp, mo=True, w=.5)
        orientConstraint(startLocation, grp, mo=True, w=.5)
        select(d=True)
        skinCluster(endAnchor, midJnt, startLocation, ribbon[0], tsb=True)
        compnentGrp = group(grp, ribbon[0], folGrp, n=name + '_Bend_Grp')

        select(d=True)
        
        multDL = shadingNode('multDoubleLinear', asUtility=True, n=name + '_MDL')
        multDL.input2.set(.5)
        endAnchor[0].rotateX >> multDL.input1
        select(d=True)
        addMembersToLayer(layer='Joint_layer', member=midJnt)
        return midJnt, multDL

    def folJntCreate(numberOfJoints):
        i = 0
        ratio = 1.0 / numberOfJoints
        folGrp = group(n=name + '_FOL_TFM')
        while i < numberOfJoints:
            # creates Follicles and attachs them to the nurbs surface
            fol = createNode('follicle', n=name + '_' + str(i) + '_FOL_Shape')
            bndJnt = joint(n=name + '_' + str(i) + '_FOL_BND', rad=.1)
            fol.getParent().rename(name + '_' + str(i) + '_FOL')
            # attaching
            ribbon[0].local >> fol.inputSurface
            ribbon[0].worldMatrix >> fol.inputWorldMatrix
            fol.outRotate >> fol.getParent().rotate
            fol.outTranslate >> fol.getParent().translate
            fol.getParent().inheritsTransform.set(False)
            # placement
            offset = ratio * i
            fol.parameterV.set(.5)
            fol.parameterU.set(ratio / 2 + offset)
            parent(fol.getParent(), folGrp)
            delete(orientConstraint(startLocation, bndJnt, mo=False))
            makeIdentity(bndJnt, a=True, r=True)
            select(cl=True)
            i += 1
            addMembersToLayer(layer='Joint_layer', member=bndJnt)

        return folGrp

    if startLocation == None or endLocation == None:
        raise RuntimeError('please provide start and end location joints')

    startPosMatrix = startLocation.getMatrix(worldSpace=True)
    endPosMatrix = endLocation.getMatrix(worldSpace=True)

    count = 0
    killOffset = -.5
    targList = []
    while count != 2:
        square = nurbsSquare(nr=(0, 1, 0))
        target = square[0].getChildren()[3]
        select(target), parent(w=True), xform(cp=True), rename(target, 'targ_' + str(count))
        target.translateX.set(killOffset)
        makeIdentity(apply=True, t=1, r=1, s=1, n=0, pn=1), delete(ch=True), delete(square)
        targList.append(target)

        count += 1

    targList[0].setMatrix(startPosMatrix)
    targList[-1].setMatrix(endPosMatrix)
    targList[-1].rotate.set(targList[0].getRotation())

    if targList[0].getTranslation(space='world')[0] < -.1:
        ribbon = loft(targList[0], targList[-1], ss=numberOfJoints, n=name, rn=True)
    else:
        ribbon = loft(targList[-1], targList[0], ss=numberOfJoints, n=name, rn=True)

    xform(cp=True), select(d=True)

    midPos = (targList[0].getTranslation(space='world') + targList[-1].getTranslation(space='world')) / 2
    delete(targList)

    folGrp = folJntCreate(numberOfJoints)
    if type == 'limb':

        return limbRibbon()
    else:
        return spineRibbon()


def getPoleVectorPosition(joints, offset, curveGuide=True):
    '''
    getPolVectorPosition
    :param name: Pass a three Joint chain list
    :param offset: set poleVector spacing
    :param CurveGuide: set To True if you want a visualization
    :return: poleVector Location
    '''
    totalJoints = len(joints)
    joints = map(nodetypes.Joint, joints)

    midpoint = dt.Vector()
    midpoint.x = sum([jnt.getTranslation('world')[0] for jnt in joints])
    midpoint.y = sum([jnt.getTranslation('world')[1] for jnt in joints])
    midpoint.z = sum([jnt.getTranslation('world')[2] for jnt in joints])
    midpoint /= totalJoints

    if totalJoints % 2 == 0:
        first = totalJoints / 2
        second = first - 1

        midJointPos = dt.Vector()
        midJointPos.x = joints[first].getTranslation('world')[0] + joints[second].getTranslation('world')[0]
        midJointPos.y = joints[first].getTranslation('world')[1] + joints[second].getTranslation('world')[1]
        midJointPos.z = joints[first].getTranslation('world')[2] + joints[second].getTranslation('world')[2]
        midJointPos /= 2
    else:
        midJointPos = dt.Vector(xform(joints[totalJoints / 2], q=True, worldSpace=True, translation=True))

    poleVector = (midJointPos - midpoint).normal()

    result = spaceLocator(n='L_oc_midchain_test')
    result.setTranslation((poleVector * offset) + midJointPos, 'world')

    if curveGuide:
        crv = curve(degree=1, point=[midpoint, (poleVector * 200) + midJointPos], k=[0, 1], n='curveGuide')
    select(d=True)

    return result


def createPoleVector(name='', poleVectorLocation=None, ikHandle=None, rotate=(0, 0, 0), **kwargs):
    '''
    createPoleVector.
    :param name: pass a name string
    :param poleVectorLocation: pass the pole vector stand in locator
    :param ikHandle: pass the ik handle to be driven
    :param rotate: rotate the ctrl shape
    :param **kwargs: pass arguments for ctrl creation

    '''
    PoleVectorPos = poleVectorLocation.getTranslation(space='world')
    ctrl = Ct.coneCtrl(name=name, **kwargs)
    ctrlGrp = ctrl.ctrl[0].getParent().getParent()
    ctrlGrp.translate.set(PoleVectorPos)
    ctrl.setRotation(rotate=rotate)
    delete(poleVectorLocation)
    poleVectorConstraint(ctrl.ctrl[0], ikHandle)
    select(d=True)
    addMembersToLayer(layer = 'Control_Layer', member= ctrl.ctrl[0])
    return ctrl.ctrl[0]


def connectAttrs(driver= None, driverAttr ='', driven = [], drivenAttr ='', reverse = False, **kwargs):
    '''
    Connects attrs.
    :param driver, Attr: pass a driver and a attribute string.
    :param driver, Attr: pass a driven and a attribute string.
    :param reverse: Mark true if you want to reverse output

    '''
    if reverse == True:

        for i in driven:
            rever = shadingNode('reverse', asUtility=True, n=i[:-9]+'_'+drivenAttr+'_reverse')
            driver.attr(driverAttr) >> rever.inputX
            rever.outputX           >> i.attr(drivenAttr)
            select(d=True)
    else:    
        for i in driven:
            driver.attr(driverAttr) >> i.attr(drivenAttr)
            


def setHandDrivenKeys(handCtrlChain):
    
    '''
    setHandDrivenKeys.
    :param handCtrlChain: pass the a list of a finger chain joints.
    '''  
     
    rot = -90
    handCtrlChain.pop(1)
    
    master = handCtrlChain[0][-1]
    driven1 = handCtrlChain[1][0]
    if len(handCtrlChain) > 2:
        driven2 = handCtrlChain[2][0]
    if handCtrlChain[0][-1].startswith('R_'):
        rot*=-1
     
    setDrivenKeyframe(driven1 + '.rz', cd= master + '.rz')
    if len(handCtrlChain) > 2:
        setDrivenKeyframe(driven2+ '.rz',  cd= master + '.rz')
        
    setAttr(master + '.rz', rot)
    setAttr(driven1 + '.rz', rot)
    if len(handCtrlChain) > 2:
        setAttr(driven2+ '.rz', rot)
        
    setDrivenKeyframe(driven1  + '.rz', cd= master + '.rz')
    if len(handCtrlChain) > 2:
        setDrivenKeyframe(driven2  + '.rz', cd= master + '.rz')
        
    setAttr(master + '.rz', 0)
    
def createFootDrv(scIks = [], rpIks = None):
    
    '''
    createFootDrv.
    :param scIks: pass a list of the two single chain solvers in the foot in ball to toe order.
    :param rpIks: pass a single IK rotateplane solver.
    '''

    pivotCtrl    =Ct.sphereCtrl(name= scIks[-1], r=.5)
    pivotCtrl.setPosition(scIks[-1])

    if scIks[0].startswith('L_'):
        pivotCtrl.setTranslation((2,0,0))
    else:
        pivotCtrl.setTranslation((-2,0,0))

    pivotCtrl.ctrlOffset.setParent(rpIks.getParent())
    select(d=True)

    grp  = group(n = scIks[0][:-13]+'_Tilt_TFM')
    grp.translate.set(scIks[-1].getTranslation('world'))
    grp.setParent(pivotCtrl.ctrl[0].getParent())
    makeIdentity(grp ,apply=True, t=1, r=1, s=1, n=0, pn=1)
    select(d=True)
    scIks[0].setParent(grp)

    ballRollCtrl =Ct.cylinderCtrl(name= scIks[-1], r = 2, hr =.3, s =4, d=3, ssw = 180, esw = 330)
    ballRollCtrl.setPosition(parent = scIks[0])

    if scIks[0].startswith('L_'):
        ballRollCtrl.setRotation((9.5,180,95))
        ballRollCtrl.setTranslation((-1.2,0,0))
    else:
        ballRollCtrl.setRotation((170.5,180,275))
        ballRollCtrl.setTranslation((1.2,0,0))
        
    ballVec = scIks[0].getTranslation('world') - ballRollCtrl.ctrl[0].getTranslation('world')
    ballRollCtrl.ctrl[0].rotatePivot.set(ballVec)
    ballRollCtrl.ctrlOffset.setParent(grp)
    rpIks.setParent(ballRollCtrl.ctrl[0])
    
    toeRollCtrl  =Ct.cylinderCtrl(name= scIks[-1], r = 2, hr =.3, s =4, d=3, ssw = 180, esw = 330)
    toeRollCtrl.setPosition(parent = scIks[0])

    if scIks[0].startswith('L_'):
        toeRollCtrl.setRotation((40,180,95))
        toeRollCtrl.setTranslation((.75,.15,.6))
    else:
        toeRollCtrl.setRotation((220,180,275))
        toeRollCtrl.setTranslation((-.75,-.15,-.6))
        toeRollCtrl.setRotation((0,-70,0))
        
    
    toeVec = scIks[0].getTranslation('world') - toeRollCtrl.ctrl[0].getTranslation('world')
    toeRollCtrl.ctrl[0].rotatePivot.set(toeVec)
    toeRollCtrl.ctrlOffset.setParent(grp)
    scIks[-1].setParent(toeRollCtrl.ctrl[0])


    pivotCtrl.ctrl[0].translate >> grp.rotatePivot
    pivotCtrl.ctrl[0].translate >> grp.scalePivot
    pivotCtrl.ctrl[0].rotate    >> grp.rotate
    select(d=True)


def attachToSubMaster(list, master):
    
    for i in list:
        i.getParent().getParent().setParent(master)
        
def jointTargViz(list = None, name='', **kwargs):
    """
    Creates a visualization for our proto type joints
    :param list: list of floating joints.
    :param name: a string that defines the name of the viz group
    :param kwargs: pass any nurbs sphere arguments through here
    :return: returns the visual hierarchy to be deleted later/
    """
    
    
    if objExists(name +'Viz_GRP'):
        vizGrp = PyNode(name+'_Viz_GRP')

    else:
        vizGrp = group(n = name+'_Viz_GRP')
        select(d=True)
        
    list = list[::-1]
    iter = 1
    for i in list:
        if iter != len(list):

            startLocation = i
            endLocation = list[iter]
            iter+=1
            crv = curve(d=1, p = [startLocation.getTranslation('world'), endLocation.getTranslation('world')])
            select(d=True)
            clust = [cluster(i) for i in crv.cv]
            select(d=True)
            hide(clust[:])
            
            clust[0][-1].setParent(startLocation)
            clust[-1][-1].setParent(endLocation)
            crv.setParent(vizGrp)

            if name == 'hand':
                startLocation.radius.set(.3)
                endLocation.radius.set(.3)
            else:
                startLocation.radius.set(1)
                endLocation.radius.set(1)

            startLocation.overrideEnabled.set(1)
            startLocation.overrideDisplayType.set(2)
            endLocation.overrideEnabled.set(1)
            endLocation.overrideDisplayType.set(2)


    ctrls =  createCtrls(type = 'sphere' ,lFkChain = list, constraint = False ,**kwargs)
    createCtrlHeiarchy(ctrls[0])
    protoVizGrp = ctrls[0][0][0]
    protoVizCtrl = [ctrls[0][0][-1], ctrls[0][0][0], ctrls[0][-1][-1]]
    vizGrp.setParent(protoVizGrp)

    select('*BUFFER'), pickWalk(d='down'), hide()
    select(d=True)

    if objExists('hand_Targ_CTRL'):
        hide('hand_Targ_CTRL')

    if objExists(name+'_Proto_GRP'):
        protoGrp = PyNode(name+'_Proto_GRP')
        protoVizGrp.setParent(protoGrp)
        vizGrp.setParent(protoGrp)
        select(d=True)
        return protoVizCtrl
    else:
        rename(protoVizGrp, name + '_Proto_GRP')
        vizGrp.setParent(protoVizGrp)
        return protoVizCtrl


def connectProtoHand(Driver, Driven):
    """

    :param driver:
    :param driven:
    :return:
    """
    driver = Driver
    for driven in Driven:
        driven = driven[0].getParent()
        parentConstraint(driver, driven, mo=True)






