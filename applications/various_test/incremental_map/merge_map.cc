#include <gflags/gflags.h>
#include <iostream>
#include <localization-summary-map/localization-summary-map.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>

#include "incremental_map/incremental_pipeline.h"

DEFINE_string(
    vimaps_to_merge, "",
    "Folders to the vi-maps which are about to be merged,"
    "use ':' to seperate individual folders; Note that the "
    "first vi-map is assumed to be the already optimized "
    "old map which is about to be updated.");

DEFINE_string(output_folder, "", "Folder to save the updated map");

DEFINE_bool(
    also_save_summary_map, true, "Generate summary map for the updated vi-map");

namespace {

void splitItemsByColon(
    const std::string& str, std::vector<std::string>* items) {
  items->clear();
  size_t start = 0;
  while (1) {
    size_t end = str.find(':', start);
    if (end != std::string::npos) {
      items->push_back(str.substr(start, end - start));
      start = end + 1;
    } else {
      items->push_back(str.substr(start));
      break;
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::vector<std::string> vimap_folders;
  std::vector<vi_map::VIMap::Ptr> vimaps_to_merge;
  if (FLAGS_vimaps_to_merge != "") {
    splitItemsByColon(FLAGS_vimaps_to_merge, &vimap_folders);
    for (auto& vimap_folder : vimap_folders) {
      vi_map::VIMap::Ptr vimap(new vi_map::VIMap());
      CHECK(vi_map::serialization::loadMapFromFolder(vimap_folder, vimap.get()))
          << "Loading VI map failed: " << vimap_folder;
      vimaps_to_merge.push_back(vimap);
    }
  } else {
    LOG(ERROR) << "the flag --vimaps_to_merge is required";
    return 1;
  }

  visualization::RVizVisualizationSink::init();
  visualization::ViwlsGraphRvizPlotter plotter;
  incremental_map::IncrementalPipeline incrementalMap(
      &vimaps_to_merge, &plotter);
  vi_map::VIMap::Ptr result_vimap = incrementalMap();
  if (result_vimap != nullptr) {
    int total_missions = 0;
    vi_map::MissionIdSet missionIds;
    result_vimap->getAllMissionIds(&missionIds);
    total_missions = missionIds.size();
    LOG(INFO) << "The merged map has " << total_missions << " missions";
    if (FLAGS_output_folder != "") {
      LOG(INFO) << "Saving merged map to " << FLAGS_output_folder;
      incrementalMap.saveResultMap(
          FLAGS_output_folder, FLAGS_also_save_summary_map);
    } else {
      LOG(WARNING) << "the flag --output_folder is not sepecified so the "
                      "result map is not saved";
    }
  } else {
    LOG(ERROR) << "Failed to merged map";
  }

  return 0;
}
