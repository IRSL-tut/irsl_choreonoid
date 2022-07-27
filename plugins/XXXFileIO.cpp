#include "XXXFileIO.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneGraph>
#include <cnoid/BodyLoader>
#include <cnoid/GeneralSceneFileImporterBase>

//#include <cnoid/MessageView>
#include <iostream>

using namespace cnoid;

namespace {
ItemFileIO* meshFileIO;
}

namespace {

class SceneFileImporter : public GeneralSceneFileImporterBase
{
public:
    SceneFileImporter() {
        setCaption("XXXBody");
        setFileTypeCaption("Scene / Mesh");
    }
    virtual Item* createItem() override {
        return new XXXBodyItem;
    }
    virtual bool load(Item* item, const std::string& filename) override {
        SgNode* shape = loadScene(filename);
        if(!shape){
            std::cerr << "load failed: " << filename << std::endl;
            return false;
        }

        auto bodyItem = static_cast<XXXBodyItem*>(item);
        BodyPtr newBody = new Body;
        newBody->rootLink()->addShapeNode(shape);
        newBody->setName("Loaded_Body");
        bodyItem->setBody(newBody);

        auto itype = currentInvocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        return true;
    }
    void setScale(double _s) { scale = _s; }
private:
    double scale;
};

}

void XXXBodyItem::registerXXXBodyItemFileIoSet(ItemManager* im)
{
    //::bodyFileIO = new BodyItemBodyFileIO;
    //im->addFileIO<BodyItem>(::bodyFileIO);

    ::meshFileIO = new SceneFileImporter;
    im->addFileIO<XXXBodyItem>(::meshFileIO);

    //im->addFileIO<BodyItem>(new StdSceneFileExporter);
    //im->addFileIO<BodyItem>(new ObjFileExporter);
}

//ItemFileIO* BodyItem::bodyFileIO()
//{
//    return ::bodyFileIO;
//}

ItemFileIO* XXXBodyItem::meshFileIO()
{
    return ::meshFileIO;
}
