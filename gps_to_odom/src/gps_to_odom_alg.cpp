#include "gps_to_odom_alg.h"

GpsToOdomAlgorithm::GpsToOdomAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

GpsToOdomAlgorithm::~GpsToOdomAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void GpsToOdomAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// GpsToOdomAlgorithm Public API
