# include "map_generator.h"


MapGenerator::MapGenerator(): geotiffCreator(new GeotiffCreator), plugin_loader_(0) ,pn_("~")
  {
    pn_.param("plugins", p_plugin_list_, std::string(""));

    std::vector<std::string> plugin_list;
    boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));

    //We always have at least one element containing "" in the string list
    if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
      plugin_loader_ = new pluginlib::ClassLoader<pandora_geotiff::MapWriterPluginInterface>(
      "pandora_geotiff", "pandora_geotiff::MapWriterPluginInterface");

      for (size_t i = 0; i < plugin_list.size(); ++i){
        try
        { 
          boost::shared_ptr<pandora_geotiff::MapWriterPluginInterface> tmp (plugin_loader_->createInstance(plugin_list[i]));
          tmp->initialize(plugin_loader_->getName(plugin_list[i]));
          plugin_vector_.push_back(tmp);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }
    }else{
      ROS_INFO("No plugins loaded for geotiff node");
    }
    
    save_mission_service = pn_.advertiseService("saveMission",&MapGenerator::saveGeotiff,this);

    ROS_INFO("Geotiff node started");
  }
 MapGenerator::~MapGenerator()
  {
    if (plugin_loader_){
      delete plugin_loader_;
    }
  }

  void MapGenerator::writeGeotiff()
  {
    
    for (size_t i = 0; i < plugin_vector_.size(); ++i){
      plugin_vector_[i]->draw(geotiffCreator);
    }
    
    geotiffCreator->onCreateGeotiffClicked();
    geotiffCreator->inCreateGeotiffClicked();
    
    geotiffCreator->saveGeotiff();
}

  bool MapGenerator::saveGeotiff(pandora_geotiff::SaveMission::Request& req ,
    pandora_geotiff::SaveMission::Response& res )
  {
    ROS_INFO("SaveMission service was requested");
    
    this->writeGeotiff();
    return true;
  }
