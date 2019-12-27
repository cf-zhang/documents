#include <iostream>
#include <string>
#include <map>
using namespace std;

#define DECLARE_CLASS() \
    protected: \
        static AttributeInfo attribute_info_; \
    public:  \
        static AttributeLayer* createAttributeLayer();

#define IMPLEMENT_CLASS(attribute_layer_name, class_name)            \
    AttributeInfo class_name::attribute_info_(attribute_layer_name,(attributeLayerConstructorFun)class_name::createAttributeLayer);\
    AttributeLayer* class_name::createAttributeLayer() \
        { return new class_name;}

class AttributeInfo;
class AttributeLayer;
typedef AttributeLayer* (*attributeLayerConstructorFun)();

class AttributeLayer
{
protected:
    AttributeLayer(){}
public:
    virtual ~AttributeLayer(){}
    static void registerAttributeInfo(AttributeInfo* ci);
    static AttributeLayer* createAttributeLayer(std::string name);
    static std::map<std::string, AttributeInfo *> *AttributeInfoMap;
};

class AttributeInfo
{
public:
    AttributeInfo(const std::string className, attributeLayerConstructorFun ctor);
    AttributeInfo();
    AttributeLayer *createAttributeLayer()const;

public:
    std::string m_className;
    attributeLayerConstructorFun m_AttributeLayerConstructor;
};

std::map<std::string, AttributeInfo *> *AttributeLayer::AttributeInfoMap = new std::map<std::string, AttributeInfo*>();

void AttributeLayer::registerAttributeInfo(AttributeInfo* ci)
{
    if (NULL != ci && AttributeInfoMap->find(ci->m_className) == AttributeInfoMap->end())
    {
        AttributeInfoMap->insert(std::map<std::string, AttributeInfo*>::value_type(ci->m_className, ci));
    }
}

AttributeLayer* AttributeLayer::createAttributeLayer(std::string name)
{
    std::map<std::string, AttributeInfo*>::const_iterator iter = AttributeInfoMap->find(name);
    if (iter != AttributeInfoMap->end())
    {
        return iter->second->createAttributeLayer();
    }
    return NULL;
}

AttributeInfo::AttributeInfo(const std::string className, attributeLayerConstructorFun ctor):m_className(className), m_AttributeLayerConstructor(ctor)
{
    AttributeLayer::registerAttributeInfo(this);
}

AttributeInfo::AttributeInfo()
{
}

AttributeLayer *AttributeInfo::createAttributeLayer()const
{
    return m_AttributeLayerConstructor ? (*m_AttributeLayerConstructor)() : 0;
}

class Test:public AttributeLayer
{
        DECLARE_CLASS()
public:
        Test(){cout << "Test constructor" << endl;}
        ~Test(){cout << "Test destructor" << endl;}
};

IMPLEMENT_CLASS("xxxx", Test)

int main()
{
    AttributeLayer* obj = AttributeLayer::createAttributeLayer("xxxx");
    delete obj;
    return 0;
}
