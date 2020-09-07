#include "ROSConstraint.h"

ROSConstraint::ROSConstraint(   QDomElement pg_root)
{
//    std::cout<<"ici on doit lire le topic"<<std::endl;
    QDomElement ElTopicName = pg_root.firstChildElement("topic_name");
    if (ElTopicName.isNull())
    {
            std::cerr<<"Error : you must defined a balise  'topic_name' for a ROS Constraint"<<std::endl;
            exit(0);
    }
    topic_name_ = ElTopicName.text().simplified().toStdString();
}
