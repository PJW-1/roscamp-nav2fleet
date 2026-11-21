#include "pode_rl_controller/onnx_policy.hpp"

#include <array>
#include <stdexcept>

namespace pode_rl_controller
{

namespace
{
std::vector<int64_t> shape_of(const Ort::TypeInfo & info)
{
  return info.GetTensorTypeAndShapeInfo().GetShape();
}
}  // namespace

OnnxPolicy::OnnxPolicy(
  const std::string & model_path, int obs_dim, int act_dim, int intra_op_threads)
: env_(ORT_LOGGING_LEVEL_WARNING, "pode_rl_controller"), obs_dim_(obs_dim), act_dim_(act_dim)
{
  Ort::SessionOptions opts;
  opts.SetIntraOpNumThreads(intra_op_threads);
  opts.SetInterOpNumThreads(1);
  opts.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  opts.AddConfigEntry("session.intra_op.allow_spinning", "0");
  opts.AddConfigEntry("session.inter_op.allow_spinning", "0");
  session_ = Ort::Session(env_, model_path.c_str(), opts);
  memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
    throw std::runtime_error(model_path + ": expected one input and one output");
  }
  Ort::AllocatorWithDefaultOptions alloc;
  input_name_ = session_.GetInputNameAllocated(0, alloc).get();
  output_name_ = session_.GetOutputNameAllocated(0, alloc).get();
  const auto in = shape_of(session_.GetInputTypeInfo(0));
  const auto out = shape_of(session_.GetOutputTypeInfo(0));
  if (in.size() != 2 || in[1] != obs_dim || out.size() != 2 || out[1] != act_dim) {
    throw std::runtime_error(model_path + ": model shape differs from the policy spec");
  }
}

std::vector<float> OnnxPolicy::run(const std::vector<float> & obs)
{
  if (static_cast<int>(obs.size()) != obs_dim_) {
    throw std::invalid_argument("observation size differs from the policy spec");
  }
  const std::array<int64_t, 2> shape{1, obs_dim_};
  Ort::Value input = Ort::Value::CreateTensor<float>(
    memory_, const_cast<float *>(obs.data()), obs.size(), shape.data(), shape.size());
  const char * in_names[] = {input_name_.c_str()};
  const char * out_names[] = {output_name_.c_str()};
  auto outputs = session_.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
  const float * a = outputs[0].GetTensorData<float>();
  return std::vector<float>(a, a + act_dim_);
}

}  // namespace pode_rl_controller
