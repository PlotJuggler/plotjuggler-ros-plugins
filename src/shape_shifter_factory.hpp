#ifndef SHAPE_SHIFTER_FACTORY_HPP
#define SHAPE_SHIFTER_FACTORY_HPP

#include <ros_type_introspection/utils/shape_shifter.hpp>

class RosIntrospectionFactory{

public:
  static RosIntrospectionFactory &get();

  static void registerMessage(const std::string& topic_name,
                              const std::string &md5sum,
                              const std::string& datatype,
                              const std::string& definition );

  static RosIntrospection::ShapeShifter* getShapeShifter(const std::string& topic_name);

  static std::vector<const std::string*> getTopicList();

  static RosIntrospection::Parser& parser()
  {
      return get()._parser;
  }

  static bool isRegistered(const std::string& topic_name);

  static void reset();

private:
  RosIntrospectionFactory() = default;
  std::map<std::string, RosIntrospection::ShapeShifter> _ss_map;
  RosIntrospection::Parser _parser;

};
//---------------------------------------------

inline RosIntrospectionFactory& RosIntrospectionFactory::get()
{
  static RosIntrospectionFactory instance;
  return instance;
}

// return true if added
inline void RosIntrospectionFactory::registerMessage(const std::string &topic_name,
                                                 const std::string &md5sum,
                                                 const std::string &datatype,
                                                 const std::string &definition)
{
    auto& instance = get();
    auto it = instance._ss_map.find(topic_name);
    if( it == instance._ss_map.end() || it->second.getMD5Sum() != md5sum )
    {
        RosIntrospection::ShapeShifter msg;
        msg.morph(md5sum, datatype,definition);
        instance._ss_map.insert( std::make_pair(topic_name, std::move(msg) ));
        parser().registerMessageDefinition( topic_name, RosIntrospection::ROSType(datatype), definition);
    }
}

inline RosIntrospection::ShapeShifter* RosIntrospectionFactory::getShapeShifter(const std::string &topic_name)
{
    auto& instance = get();
    auto it = instance._ss_map.find( topic_name );
    return ( it == instance._ss_map.end()) ? nullptr :  &(it->second);
}

inline std::vector<const std::string*> RosIntrospectionFactory::getTopicList()
{
    std::vector<const std::string*> out;
    auto& instance = get();
    out.reserve( instance._ss_map.size() );

    for (const auto& ss: instance._ss_map)
    {
        out.push_back( &(ss.first) );
    }
    return out;
}

inline bool RosIntrospectionFactory::isRegistered(const std::string &topic_name)
{
    return get()._ss_map.count(topic_name) != 0;
}

inline void RosIntrospectionFactory::reset()
{
    get()._ss_map.clear();
}

#endif // SHAPE_SHIFTER_FACTORY_HPP



