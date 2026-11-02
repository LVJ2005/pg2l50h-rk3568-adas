#include "../../includes/recognition_control.h"
#include <cassert>
#include <iostream>
#include <string>

int main() {
  RecognitionRuntimeConfig config;
  config.set_plate_model_available(PlateModelProfile::Baseline, true);
  config.set_plate_model_available(PlateModelProfile::Int8, true);
  config.set_plate_model_available(PlateModelProfile::Hybrid, false);
  std::string response;
  assert(config.apply_command("MODEL int8", response));
  assert(config.current_plate_model() == PlateModelProfile::Int8);
  assert(response.find("plate_model=int8") != std::string::npos);
  const auto generation = config.generation.load();
  assert(!config.apply_command("MODEL hybrid", response));
  assert(config.current_plate_model() == PlateModelProfile::Int8);
  assert(config.generation.load() == generation);
  config.mark_plate_model_loaded(PlateModelProfile::Int8);
  assert(config.apply_command("STATUS", response));
  assert(response.find("loaded_plate_model=int8") != std::string::npos);
  assert(response.find("available_models=baseline,int8") != std::string::npos);
  std::cout << response << std::endl;
  return 0;
}
