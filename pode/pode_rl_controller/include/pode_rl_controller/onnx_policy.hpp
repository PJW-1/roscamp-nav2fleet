#pragma once

#include <onnxruntime_cxx_api.h>

#include <string>
#include <vector>

namespace pode_rl_controller
{

// ---- ONNX Runtime 정책(CPU, 한 번에 관측 하나) ----
class OnnxPolicy
{
public:
  OnnxPolicy(const std::string & model_path, int obs_dim, int act_dim, int intra_op_threads);
  std::vector<float> run(const std::vector<float> & obs);

private:
  Ort::Env env_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo memory_{nullptr};
  std::string input_name_;
  std::string output_name_;
  int obs_dim_;
  int act_dim_;
};

}  // namespace pode_rl_controller
