#include "btDeformableMultiBodyWorldImporter.h"

#include "LinearMath/btSerializer.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyData.h"
#include "../BulletFileLoader/btBulletFile.h"

struct btDeformableMultiBodyWorldImporterInternalData
{
    btDeformableMultiBodyDynamicsWorld* m_dmbDynamicsWorld;
    // Used in convert logic path
//    btHashMap<btHashPtr, btMultiBody*> m_mbMap;
};

btDeformableMultiBodyWorldImporter::btDeformableMultiBodyWorldImporter(btDeformableMultiBodyDynamicsWorld *world)
    : btMultiBodyWorldImporter(world)
{
    m_deform_data = new btDeformableMultiBodyWorldImporterInternalData;
    m_deform_data->m_dmbDynamicsWorld = world;
}

btDeformableMultiBodyWorldImporter::~btDeformableMultiBodyWorldImporter()
{
    delete m_deform_data;
}

bool checkInput(const int expected, const int received, const char func[], const char name[])
{
    const bool match = expected == received;
    if (!match)
    {
        printf("%s error: expected %d %s, got %d.\n", func, expected, name, received);
    }
    return match;
}

bool checkInput(const float expected, const float received, const char func[], const char name[])
{
    const bool match = expected == received;
    if (!match)
    {
        printf("%s error: expected %f %s, got %f.\n", func, expected, name, received);
    }
    return match;
}


bool syncCollisionObject(const btCollisionObjectFloatData& obd, btSoftBody* sb)
{
    bool success = true;

    btTransform world_transform;
    world_transform.deSerializeFloat(obd.m_worldTransform);
    sb->setWorldTransform(world_transform);

    btTransform interpolationWorldTransform;
    world_transform.deSerializeFloat(obd.m_interpolationWorldTransform);
    sb->setInterpolationWorldTransform(interpolationWorldTransform);

    btVector3 interpolationLinearVelocity;
    interpolationLinearVelocity.deSerializeFloat(obd.m_interpolationLinearVelocity);
    sb->setInterpolationLinearVelocity(interpolationLinearVelocity);

    btVector3 interpolationAngularVelocity;
    interpolationLinearVelocity.deSerializeFloat(obd.m_interpolationAngularVelocity);
    sb->setInterpolationAngularVelocity(interpolationLinearVelocity);

    // TODO: checks/values commented out are not (yet) recorded/loaded in a way that allows for comparison

    // Anistropic friction is not recorded/loaded correctly in the data, so skip it and hope for the best
//    btVector3 anisotropicFriction;
//    interpolationLinearVelocity.deSerializeFloat(obd.m_anisotropicFriction);
//    success &= checkInput((float)sb->getAnisotropicFriction().x(), (float)anisotropicFriction.x(), __func__, "anistropicFriction.x");
//    success &= checkInput((float)sb->getAnisotropicFriction().y(), (float)anisotropicFriction.y(), __func__, "anistropicFriction.y");
//    success &= checkInput((float)sb->getAnisotropicFriction().z(), (float)anisotropicFriction.z(), __func__, "anistropicFriction.z");

//    success &= checkInput(sb->m_hasAnisotropicFriction,          obd.m_hasAnisotropicFriction,       __func__, "hasAnistropicFriction");
    success &= checkInput(sb->getContactProcessingThreshold(),   obd.m_contactProcessingThreshold,   __func__, "contactProcessingThreshold");
//    success &= checkInput(sb->getBroadphaseHandle(),             obd.m_broadphaseHandle,             __func__, "broadphaseHandle");
//    success &= checkInput(sb->getCollisionShape(),               obd.m_collisionShape,               __func__, "collisionShape");
//    success &= checkInput(sb->???,                               obd.m_rootCollisionShape,           __func__, "rootCollisionShape");
    success &= checkInput(sb->getCollisionFlags(),               obd.m_collisionFlags,               __func__, "collisionFlags");
//    success &= checkInput(sb->getIslandTag(),                    obd.m_islandTag1,                   __func__, "islandTag");
    success &= checkInput(sb->getCompanionId(),                  obd.m_companionId,                  __func__, "companionId");
    success &= checkInput(sb->getActivationState(),              obd.m_activationState1,             __func__, "activationState1");
    success &= checkInput(sb->getDeactivationTime(),             obd.m_deactivationTime,             __func__, "deactivationTime");
    success &= checkInput(sb->getFriction(),                     obd.m_friction,                     __func__, "friction");
    success &= checkInput(sb->getRollingFriction(),              obd.m_rollingFriction,              __func__, "rollingFriction");
    success &= checkInput(sb->getContactDamping(),               obd.m_contactDamping,               __func__, "contactDamping");
    success &= checkInput(sb->getContactStiffness(),             obd.m_contactStiffness,             __func__, "contactStiffness");
    success &= checkInput(sb->getRestitution(),                  obd.m_restitution,                  __func__, "restitution");
    success &= checkInput(sb->getInternalType(),                 obd.m_internalType,                 __func__, "internalType");
    success &= checkInput(sb->getHitFraction(),                  obd.m_hitFraction,                  __func__, "hitFraction");
    success &= checkInput(sb->getCcdSweptSphereRadius(),         obd.m_ccdSweptSphereRadius,         __func__, "ccdSweptSphereRadius");
    success &= checkInput(sb->getCcdMotionThreshold(),           obd.m_ccdMotionThreshold,           __func__, "ccdMotionThreshold");
//    success &= checkInput(sb->m_checkCollideWith,                obd.m_checkCollideWith,             __func__, "checkCollideWidth");

    // TODO: //	char					*m_name;

    if (sb->getBroadphaseHandle())
    {
        success &= checkInput(sb->getBroadphaseHandle()->m_collisionFilterGroup, obd.m_collisionFilterGroup, __func__, "collisionFilterGroup");
        success &= checkInput(sb->getBroadphaseHandle()->m_collisionFilterMask,  obd.m_collisionFilterMask,  __func__, "collisionFilterMask");
        success &= checkInput(sb->getBroadphaseHandle()->m_uniqueId,             obd.m_uniqueId,             __func__, "uniqueId");
    }

    return success;
}

bool checkConfig(const SoftBodyConfigData& config_data, const btSoftBody::Config& config)
{
    bool success = true;
    success &= checkInput(config.kDF,         config_data.m_dynamicFriction,                __func__, "dynamicFriction");
    success &= checkInput(config.kVCF,        config_data.m_baumgarte,                      __func__, "baumgarte");
    success &= checkInput(config.kPR,         config_data.m_pressure,                       __func__, "pressure");
    success &= checkInput(config.aeromodel,   config_data.m_aeroModel,                      __func__, "aeroModel");
    success &= checkInput(config.kLF,         config_data.m_lift,                           __func__, "lift");
    success &= checkInput(config.kDG,         config_data.m_drag,                           __func__, "drag");
    success &= checkInput(config.piterations, config_data.m_positionIterations,             __func__, "positionIterations");
    success &= checkInput(config.diterations, config_data.m_driftIterations,                __func__, "driftIterations");
    success &= checkInput(config.citerations, config_data.m_clusterIterations,              __func__, "clusterIterations");
    success &= checkInput(config.viterations, config_data.m_velocityIterations,             __func__, "dynamicvelocityIterations");
    success &= checkInput(config.maxvolume,   config_data.m_maxVolume,                      __func__, "maxVolume");
    success &= checkInput(config.kDP,         config_data.m_damping,                        __func__, "damping");
    success &= checkInput(config.kMT,         config_data.m_poseMatch,                      __func__, "poseMatch");
    success &= checkInput(config.collisions,  config_data.m_collisionFlags,                 __func__, "collisionFlags");
    success &= checkInput(config.kVC,         config_data.m_volume,                         __func__, "volume");
    success &= checkInput(config.kCHR,        config_data.m_rigidContactHardness,           __func__, "rigidContactHardness");
    success &= checkInput(config.kKHR,        config_data.m_kineticContactHardness,         __func__, "kineticContactHardness");
    success &= checkInput(config.kSHR,        config_data.m_softContactHardness,            __func__, "softContactHardness");
    success &= checkInput(config.kAHR,        config_data.m_anchorHardness,                 __func__, "anchorHardness");
    success &= checkInput(config.timescale,   config_data.m_timeScale,                      __func__, "timeScale");
    success &= checkInput(config.kSRHR_CL,    config_data.m_softRigidClusterHardness,       __func__, "softRigidClusterHardness");
    success &= checkInput(config.kSKHR_CL,    config_data.m_softKineticClusterHardness,     __func__, "softKineticClusterHardness");
    success &= checkInput(config.kSSHR_CL,    config_data.m_softSoftClusterHardness,        __func__, "softSoftClusterHardness");
    success &= checkInput(config.kSR_SPLT_CL, config_data.m_softRigidClusterImpulseSplit,   __func__, "softRigidClusterHardness");
    success &= checkInput(config.kSK_SPLT_CL, config_data.m_softKineticClusterImpulseSplit, __func__, "softKineticClusterImpulseSplit");
    success &= checkInput(config.kSS_SPLT_CL, config_data.m_softSoftClusterImpulseSplit,    __func__, "softSoftClusterImpulseSplit");
    return success;
}

bool syncNodes(const SoftBodyNodeData* node_data, btSoftBody::tNodeArray& nodes)
{
    bool success = true;
    for (int i = 0; i < nodes.size(); i++)
    {
        // Error check the static data
        success &= checkInput((float)nodes[i].m_area, node_data[i].m_area,        __func__, "nodes.area");
        success &= checkInput(nodes[i].m_battach,     node_data[i].m_attach,      __func__, "nodes.attach");
        success &= checkInput((float)nodes[i].m_im,   node_data[i].m_inverseMass, __func__, "nodes.inverseMass");

        nodes[i].m_f.deSerializeFloat(node_data[i].m_accumulatedForce);
        nodes[i].m_n.deSerializeFloat(node_data[i].m_normal);
        nodes[i].m_x.deSerializeFloat(node_data[i].m_position);
        nodes[i].m_q.deSerializeFloat(node_data[i].m_previousPosition);
        nodes[i].m_v.deSerializeFloat(node_data[i].m_velocity);
    }
    return success;
}

bool checkLinks(const SoftBodyLinkData* link_data, btSoftBody::tLinkArray& links, const btSoftBody::Node* node0)
{
    bool success = true;
    // Nothing to be loaded when syncing links, just verifying data
    for (int i = 0; i < links.size(); i++)
    {
        success &= checkInput(links[i].m_bbending,     link_data[i].m_bbending,       __func__, "links.bending");
        // TODO: material
        success &= checkInput(links[i].m_n[0] - node0, link_data[i].m_nodeIndices[0], __func__, "links.nodeIndices[0]");
        success &= checkInput(links[i].m_n[1] - node0, link_data[i].m_nodeIndices[1], __func__, "links.nodeIndices[1]");
        success &= checkInput(links[i].m_rl,           link_data[i].m_restLength,     __func__, "links.restLength");
    }
    return success;
}

bool syncFaces(const SoftBodyFaceData* face_data, btSoftBody::tFaceArray& faces, const btSoftBody::Node* node0)
{
    bool success = true;
    for (int i = 0; i < faces.size(); i++)
    {
        // TODO: material
        faces[i].m_normal.deSerializeFloat(face_data[i].m_normal);
        success &= checkInput(faces[i].m_n[0] - node0, face_data[i].m_nodeIndices[0], __func__, "face.nodeIndices[0]");
        success &= checkInput(faces[i].m_n[1] - node0, face_data[i].m_nodeIndices[1], __func__, "face.nodeIndices[1]");
        success &= checkInput(faces[i].m_n[2] - node0, face_data[i].m_nodeIndices[2], __func__, "face.nodeIndices[2]");
        success &= checkInput(faces[i].m_ra,           face_data[i].m_restArea,       __func__, "face.restArea");
    }
    return success;
}

bool checkTetrahedra(const SoftBodyTetraData* tetra_data, btSoftBody::tTetraArray& tetras, const btSoftBody::Node* node0)
{
    bool success = true;
    for (int i = 0; i < tetras.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            success &= checkInput(tetras[i].m_c0[j].x(),     tetra_data[i].m_c0[j].m_floats[0], __func__, "tetra.c0.x");
            success &= checkInput(tetras[i].m_c0[j].y(),     tetra_data[i].m_c0[j].m_floats[1], __func__, "tetra.c0.y");
            success &= checkInput(tetras[i].m_c0[j].z(),     tetra_data[i].m_c0[j].m_floats[2], __func__, "tetra.c0.z");
            success &= checkInput(tetras[i].m_n[j] - node0,  tetra_data[i].m_nodeIndices[j],    __func__, "tetra.nodeIndices");
        }

        success &= checkInput(tetras[i].m_c1, tetra_data[i].m_c1,         __func__, "tetra.c1");
        success &= checkInput(tetras[i].m_c2, tetra_data[i].m_c2,         __func__, "tetra.c2");
        // TODO: material
        success &= checkInput(tetras[i].m_rv, tetra_data[i].m_restVolume, __func__, "tetra.restVolume");
    }
    return success;
}

bool syncAnchors(const SoftRigidAnchorData* anchor_data, btSoftBody::tAnchorArray& anchors, const btSoftBody::Node* node0)
{
    bool success = true;
    for (int i = 0; i < anchors.size(); i++)
    {
        anchors[i].m_c0.deSerializeFloat(anchor_data[i].m_c0);
        anchors[i].m_c1.deSerializeFloat(anchor_data[i].m_c1);
        anchors[i].m_c2 = anchor_data[i].m_c2;
        anchors[i].m_local.deSerializeFloat(anchor_data[i].m_localFrame);
        // TODO: rigid body
        success &= checkInput(anchors[i].m_node - node0, anchor_data[i].m_nodeIndex, __func__, "anchors.nodeIndex");
    }
    return success;
}

bool syncPose(const SoftBodyPoseData* pose_data, btSoftBody::Pose& pose)
{
    bool success = true;
    success &= checkInput(pose.m_pos.size(), pose_data->m_numPositions, __func__, "pose.numPositions");
    success &= checkInput(pose.m_wgh.size(), pose_data->m_numWeigts,    __func__, "pose.numWeights");
    if (!success)
    {
        return success;
    }

    pose.m_aqq.deSerializeFloat(pose_data->m_aqq);
    success &= checkInput(pose.m_bframe,  pose_data->m_bframe,  __func__, "pose.isFrame");
    success &= checkInput(pose.m_bvolume, pose_data->m_bvolume, __func__, "pose.isValid");
    pose.m_com.deSerializeFloat(pose_data->m_com);
    for (int i = 0; i < pose.m_pos.size(); i++)
    {
        pose.m_pos[i].deSerializeFloat(pose_data->m_positions[i]);
    }
    success &= checkInput(pose.m_volume, pose_data->m_restVolume, __func__, "pose.restVolume");
    pose.m_rot.deSerializeFloat(pose_data->m_rot);
    pose.m_scl.deSerializeFloat(pose_data->m_scale);
    for (int i = 0; i < pose.m_wgh.size(); i++)
    {
        pose.m_wgh[i] = pose_data->m_weights[i];
    }

    return success;
}

bool syncClusters(const SoftBodyClusterData* cluster_data, btSoftBody::tClusterArray& clusters)
{
    bool success = true;
    for (int i = 0; i < clusters.size(); i++)
    {
        success &= checkInput(clusters[i]->m_framerefs.size(), cluster_data->m_numFrameRefs, __func__, "clusters.numFrameRefs");
        success &= checkInput(clusters[i]->m_nodes.size(),     cluster_data->m_numNodes,     __func__, "clusters.numNodes");
        success &= checkInput(clusters[i]->m_masses.size(),    cluster_data->m_numMasses,    __func__, "clusters.numMasses");
        if (!success)
        {
            return success;
        }

        clusters[i]->m_av.deSerializeFloat(cluster_data[i].m_av);
        success &= checkInput(clusters[i]->m_clusterIndex,   cluster_data[i].m_clusterIndex,   __func__, "clusters.clusterIndex");
        success &= checkInput(clusters[i]->m_collide,        cluster_data[i].m_collide,        __func__, "clusters.collide");
        clusters[i]->m_com.deSerializeFloat(cluster_data[i].m_com);
        success &= checkInput(clusters[i]->m_containsAnchor, cluster_data[i].m_containsAnchor, __func__, "clusters.containsAnchor");
        clusters[i]->m_dimpulses[0].deSerializeFloat(cluster_data[i].m_dimpulses[0]);
        clusters[i]->m_dimpulses[1].deSerializeFloat(cluster_data[i].m_dimpulses[1]);
        clusters[i]->m_framexform.deSerializeFloat(cluster_data[i].m_framexform);
        success &= checkInput(clusters[i]->m_idmass,         cluster_data[i].m_idmass,         __func__, "clusters.idmass");
        success &= checkInput(clusters[i]->m_imass,          cluster_data[i].m_imass,          __func__, "clusters.imass");
        clusters[i]->m_invwi.deSerializeFloat(cluster_data[i].m_invwi);
        clusters[i]->m_locii.deSerializeFloat(cluster_data[i].m_locii);
        clusters[i]->m_lv.deSerializeFloat(cluster_data[i].m_lv);
        success &= checkInput(clusters[i]->m_matching,       cluster_data[i].m_matching,       __func__, "clusters.matching");
        success &= checkInput(clusters[i]->m_maxSelfCollisionImpulse, cluster_data[i].m_maxSelfCollisionImpulse, __func__, "clusters.maxSelfCollisionImpulse");
        success &= checkInput(clusters[i]->m_ndamping,       cluster_data[i].m_ndamping,       __func__, "clusters.nodeDamping");
        success &= checkInput(clusters[i]->m_ldamping,       cluster_data[i].m_ldamping,       __func__, "clusters.linearDamping");
        success &= checkInput(clusters[i]->m_adamping,       cluster_data[i].m_adamping,       __func__, "clusters.angularDamping");
        success &= checkInput(clusters[i]->m_selfCollisionImpulseFactor, cluster_data[i].m_selfCollisionImpulseFactor, __func__, "clusters.selfCollisionImpulseFactor");
        clusters[i]->m_nvimpulses = cluster_data[i].m_nvimpulses;
        clusters[i]->m_vimpulses[0].deSerializeFloat(cluster_data[i].m_vimpulses[0]);
        clusters[i]->m_vimpulses[1].deSerializeFloat(cluster_data[i].m_vimpulses[1]);
        clusters[i]->m_ndimpulses = cluster_data[i].m_ndimpulses;

        for (int i = 0; i < clusters[i]->m_framerefs.size(); i++)
        {
            clusters[i]->m_framerefs[i].deSerializeFloat(cluster_data[i].m_framerefs[i]);
        }
        for (int i = 0; i < clusters[i]->m_masses.size(); i++)
        {
            clusters[i]->m_masses[i] = cluster_data[i].m_masses[i];
        }
        for (int i = 0; i < clusters[i]->m_nodes.size(); i++)
        {
            // TODO
        }
    }
    return success;
}

bool syncJoints(const btSoftBodyJointData* joint_data, btSoftBody::tJointArray& joints)
{
    bool success = true;
    for (int i = 0; i < joints.size(); i++)
    {
        success &= checkInput(joints[i]->Type(), joint_data[i].m_jointType, __func__, "joints.jointType");
        joints[i]->m_refs[0].deSerializeFloat(joint_data[i].m_refs[0]);
        joints[i]->m_refs[1].deSerializeFloat(joint_data[i].m_refs[1]);
        success &= checkInput(joints[i]->m_cfm, joint_data[i].m_cfm, __func__, "joints.cfm");
        success &= checkInput(joints[i]->m_erp, joint_data[i].m_erp, __func__, "joints.erp");
        success &= checkInput(joints[i]->m_split, joint_data[i].m_split, __func__, "joints.split");
        success &= checkInput(joints[i]->m_delete, joint_data[i].m_delete, __func__, "joints.delete");
        // TODO: m_relPositions
        // TODO: m_bodyA
        // TODO: m_bodyB
        // TODO: m_bodyAType
        // TODO: m_bodyBType
    }
    return success;
}

// Deserialization based on serialization from btSoftBody::serialize(...)
bool syncSoftBody(btSoftBodyFloatData* sbd, btSoftBody* sb, btDeformableMultiBodyDynamicsWorld* world)
{
    bool success = syncCollisionObject(sbd->m_collisionObjectData, sb);

    // Error check the top level data - i.e. is this the same object?
    success &= checkInput(sb->m_materials.size(), sbd->m_numMaterials,  __func__, "materials");
    success &= checkInput(sb->m_nodes.size(),     sbd->m_numNodes,      __func__, "nodes");
    success &= checkInput(sb->m_links.size(),     sbd->m_numLinks,      __func__, "links");
    success &= checkInput(sb->m_faces.size(),     sbd->m_numFaces,      __func__, "faces");
    success &= checkInput(sb->m_tetras.size(),    sbd->m_numTetrahedra, __func__, "tetrahedra");
    success &= checkInput(sb->m_anchors.size(),   sbd->m_numAnchors,    __func__, "anchors");
    success &= checkInput(sb->m_clusters.size(),  sbd->m_numClusters,   __func__, "clusters");
    success &= checkInput(sb->m_joints.size(),    sbd->m_numJoints,     __func__, "joints");
    if (!success)
    {
        return success;
    }

    // Error check the materials data - i.e. is this the same object?
    // TODO

    success &= checkConfig(sbd->m_config, sb->m_cfg);
    success &= syncNodes(sbd->m_nodes, sb->m_nodes);
    success &= checkLinks(sbd->m_links, sb->m_links, &sb->m_nodes[0]);
    success &= syncFaces(sbd->m_faces, sb->m_faces, &sb->m_nodes[0]);
    success &= checkTetrahedra(sbd->m_tetrahedra, sb->m_tetras, &sb->m_nodes[0]);
    success &= syncAnchors(sbd->m_anchors, sb->m_anchors, &sb->m_nodes[0]);
    success &= syncPose(sbd->m_pose, sb->m_pose);
    success &= syncClusters(sbd->m_clusters, sb->m_clusters);
    success &= syncJoints(sbd->m_joints, sb->m_joints);

    return success;
}

void addSoftBody(btSoftBodyFloatData* sbd, btDeformableMultiBodyDynamicsWorld* world)
{
    (void)sbd;
    (void)world;
    assert(false && "Not implemented");
}

bool btDeformableMultiBodyWorldImporter::convertAllObjects(bParse::btBulletFile* bulletFile2)
{
    // First convert all MultiBodyWorld objects;
    bool success = btMultiBodyWorldImporter::convertAllObjects(bulletFile2);

    // Next convert the soft bodies
    if (m_importerFlags & eRESTORE_EXISTING_OBJECTS)
    {
        if (bulletFile2->m_softBodies.size() != m_deform_data->m_dmbDynamicsWorld->getSoftBodyArray().size())
        {
            printf("btDeformableMultiBodyWorldImporter::convertAllObjects error: expected %d softbodies, got %d.\n",
                   m_deform_data->m_dmbDynamicsWorld->getSoftBodyArray().size(), bulletFile2->m_softBodies.size());
            success = false;
            return success;
        }

        for (int i = 0; i < bulletFile2->m_softBodies.size(); i++)
        {
            btSoftBodyFloatData* sbd = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
            btSoftBody* sb = m_deform_data->m_dmbDynamicsWorld->getSoftBodyArray()[i];
            success = syncSoftBody(sbd, sb, m_deform_data->m_dmbDynamicsWorld);
            if (!success)
            {
                return success;
            }
        }
    }
    else
    {
        for (int i = 0; i < bulletFile2->m_softBodies.size(); i++)
        {
            btSoftBodyData* sbObjData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
            addSoftBody(sbObjData, m_deform_data->m_dmbDynamicsWorld);
        }
    }

    return success;
}

void btDeformableMultiBodyWorldImporter::deleteAllData()
{
        btMultiBodyWorldImporter::deleteAllData();
}

