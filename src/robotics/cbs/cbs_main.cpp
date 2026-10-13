/*
 * cbs_main.cpp
 *
 * Created on: Oct 04, 2026 12:00
 * Description: CLI for running CBS on movingai.com benchmark instances.
 *
 * Usage:
 *   cbs_main -m <map> -a <agents> -k <num_agents> [-o output] [-p paths]
 *            [-t cutoff_sec] [-s screen]
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "algorithm/robotics/cbs/cbs.hpp"
#include "algorithm/robotics/cbs/instance.hpp"

using namespace pllee4::graph;

namespace {
struct Options {
  std::string map_file;
  std::string agent_file;
  std::string output_file;
  std::string paths_file;
  int num_agents{0};
  int cutoff_sec{60};
  int screen{1};
};

void PrintUsage(const char *prog) {
  std::cerr
      << "Usage: " << prog << " [options]\n"
      << "  -m, --map <file>       input .map file (required)\n"
      << "  -a, --agents <file>    input .scen file (required)\n"
      << "  -k, --agentNum <n>     number of agents (required)\n"
      << "  -o, --output <file>    output CSV for results\n"
      << "  -p, --outputPaths <f>  output file for paths\n"
      << "  -t, --cutoffTime <s>   cutoff in seconds (default 60)\n"
      << "  -s, --screen <0|1|2>   verbosity (default 1)\n";
}

bool ParseArgs(int argc, char **argv, Options &opts) {
  for (int i = 1; i < argc; ++i) {
    auto match = [&](const char *s, const char *l) {
      return std::strcmp(argv[i], s) == 0 ||
             std::strcmp(argv[i], l) == 0;
    };
    if (match("-m", "--map") && i + 1 < argc)
      opts.map_file = argv[++i];
    else if (match("-a", "--agents") && i + 1 < argc)
      opts.agent_file = argv[++i];
    else if (match("-k", "--agentNum") && i + 1 < argc)
      opts.num_agents = std::atoi(argv[++i]);
    else if (match("-o", "--output") && i + 1 < argc)
      opts.output_file = argv[++i];
    else if (match("-p", "--outputPaths") && i + 1 < argc)
      opts.paths_file = argv[++i];
    else if (match("-t", "--cutoffTime") && i + 1 < argc)
      opts.cutoff_sec = std::atoi(argv[++i]);
    else if (match("-s", "--screen") && i + 1 < argc)
      opts.screen = std::atoi(argv[++i]);
    else if (match("-h", "--help")) {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown option: " << argv[i] << '\n';
      return false;
    }
  }
  return !opts.map_file.empty() && !opts.agent_file.empty() &&
         opts.num_agents > 0;
}
}  // namespace

int main(int argc, char **argv) {
  auto opts = Options{};
  if (!ParseArgs(argc, argv, opts)) {
    PrintUsage(argv[0]);
    return 1;
  }

  // Load instance.
  auto instance = Instance{};
  if (!instance.LoadMap(opts.map_file)) {
    std::cerr << "Failed to load map: " << opts.map_file << '\n';
    return 1;
  }
  if (!instance.LoadAgents(opts.agent_file, opts.num_agents)) {
    std::cerr << "Failed to load agents: " << opts.agent_file << '\n';
    return 1;
  }

  if (opts.screen >= 2) {
    instance.PrintMap();
    instance.PrintAgents();
  }

  // Solve.
  auto cbs = CBS{instance.width(), instance.height(), instance.obstacles(),
                 instance.agents()};

  const auto t0 = std::chrono::steady_clock::now();
  auto result = cbs.Solve(opts.cutoff_sec * 1000);
  const auto t1 = std::chrono::steady_clock::now();
  const auto runtime_s =
      std::chrono::duration<double>(t1 - t0).count();

  // Output.
  if (opts.screen >= 1) {
    Instance::PrintResults(result, runtime_s);
  }
  if (result.solved && opts.screen >= 2) {
    Instance::PrintPaths(result.solution, instance.width());
  }

  if (!opts.output_file.empty()) {
    const auto name = opts.agent_file + ":" +
                      std::to_string(opts.num_agents);
    Instance::SaveResults(opts.output_file, result, runtime_s, name);
  }
  if (result.solved && !opts.paths_file.empty()) {
    Instance::SavePaths(opts.paths_file, result.solution);
  }

  return result.solved ? 0 : 1;
}
