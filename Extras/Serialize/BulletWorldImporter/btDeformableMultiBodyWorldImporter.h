#ifndef BT_DEFORMABLE_MULTIBODY_WORLD_IMPORTER_H
#define BT_DEFORMABLE_MULTIBODY_WORLD_IMPORTER_H

#include "btMultiBodyWorldImporter.h"

class btDeformableMultiBodyWorldImporter : public btMultiBodyWorldImporter
{
    struct btDeformableMultiBodyWorldImporterInternalData* m_deform_data;

public:
    btDeformableMultiBodyWorldImporter(class btDeformableMultiBodyDynamicsWorld* world);
    virtual ~btDeformableMultiBodyWorldImporter();

    virtual bool convertAllObjects(bParse::btBulletFile* bulletFile2);

    virtual void deleteAllData();
};

#endif // BT_DEFORMABLE_MULTIBODY_WORLD_IMPORTER_H
