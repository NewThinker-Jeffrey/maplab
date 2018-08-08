#include "map-anchoring-plugin/anchoring-plugin.h"

#include <console-common/console.h>
#include <map-anchoring/map-anchoring.h>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>

DECLARE_string(map_mission);
DECLARE_string(another_vimap);
DECLARE_string(another_summary_map);

namespace map_anchoring_plugin {

AnchoringPlugin::AnchoringPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(CHECK_NOTNULL(console), plotter) {
  addCommand(
      {"set_mission_baseframe_to_known", "sbk"},
      [this]() -> int {
        constexpr bool kIsKnown = true;
        return setMissionBaseframeKnownState(kIsKnown);
      },
      "Mark mission baseframe of a mission as known.",
      common::Processing::Sync);
  addCommand(
      {"set_mission_baseframe_to_unknown"},
      [this]() -> int {
        constexpr bool kIsKnown = false;
        return setMissionBaseframeKnownState(kIsKnown);
      },
      "Mark mission baseframe of a mission as unknown.",
      common::Processing::Sync);

  addCommand(
      {"set_all_mission_baseframes_to_unknown", "sabu"},
      [this]() -> int {
        constexpr bool kIsKnown = false;
        return setAllMissionBaseframesKnownState(kIsKnown);
      },
      "Mark all mission baseframes as unknown.", common::Processing::Sync);
  addCommand(
      {"set_all_mission_baseframes_to_known"},
      [this]() -> int {
        constexpr bool kIsKnown = true;
        return setAllMissionBaseframesKnownState(kIsKnown);
      },
      "Mark all mission baseframes as known.", common::Processing::Sync);

  addCommand(
      {"anchor_mission", "am"}, [this]() -> int { return anchorMission(); },
      "Try to anchor this mission to another mission with known baseframe.",
      common::Processing::Sync);

  addCommand(
      {"anchor_all_missions", "aam"},
      [this]() -> int { return anchorAllMissions(); },
      "Try to anchor all missions to another mission with known baseframe.",
      common::Processing::Sync);

  addCommand(
      {"reanchor_to_another_map_with_common_landmarks",
       "reanchor_common_landmarks", "racl"},
      [this]() -> int { return alignLandmarksToAnotherMap(); },
      "This command is useful if you need to merge an existed map "
      "with some newly collected datasets to update the map and "
      "you wish the merged map using the consistent baseframes "
      "with the original map. This command would firstly find "
      "the common landmarks between the selected map and the other "
      "map you specified in command line, "
      "calculate a best global transformation mapping the "
      "common landmarks from the selected map to the other map, and "
      "then apply it on the selected map. You should use "
      "--another_vimap or --another_summary_map to specify "
      "the map you want to align the selected map to.",
      common::Processing::Sync);
}

int AnchoringPlugin::setMissionBaseframeKnownState(
    const bool baseframe_known_state) const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  if (!mission_id.isValid()) {
    LOG(ERROR) << "The given mission \"" << FLAGS_map_mission
               << "\" is not valid.";
    return common::kUnknownError;
  }

  map_anchoring::setMissionBaseframeKnownState(
      mission_id, baseframe_known_state, map.get());

  return common::kSuccess;
}

int AnchoringPlugin::setAllMissionBaseframesKnownState(
    const bool baseframe_known_state) const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  vi_map::MissionIdList all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    map_anchoring::setMissionBaseframeKnownState(
        mission_id, baseframe_known_state, map.get());
  }

  return common::kSuccess;
}

int AnchoringPlugin::anchorMission() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  if (!mission_id.isValid()) {
    LOG(ERROR) << "The given mission \"" << FLAGS_map_mission
               << "\" is not valid.";
    return common::kUnknownError;
  }

  map_anchoring::anchorMission(mission_id, map.get());
  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  return common::kSuccess;
}

int AnchoringPlugin::anchorAllMissions() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  const bool success = map_anchoring::anchorAllMissions(map.get());
  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  return success ? common::kSuccess : common::kUnknownError;
}

extern bool alignLandmarksToAnotherVIMap(std::string selected_map_key);
extern bool alignLandmarksToAnotherSummaryMap(std::string selected_map_key);
int AnchoringPlugin::alignLandmarksToAnotherMap() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  if (FLAGS_another_vimap != "") {
    if (alignLandmarksToAnotherVIMap(selected_map_key))
      return common::kSuccess;
  } else if (FLAGS_another_summary_map != "") {
    if (alignLandmarksToAnotherSummaryMap(selected_map_key))
      return common::kSuccess;
  } else {
    LOG(ERROR)
        << "Neither --another_vimap nor --another_summary_map is specified";
    return common::kStupidUserError;
  }
  return common::kUnknownError;
}

}  // namespace map_anchoring_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    map_anchoring_plugin::AnchoringPlugin);
