// Host-only test double. This file is never linked into the production app.
#include "rknn_api.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

namespace {
struct Model {bool detector;std::vector<std::vector<float>> output;};
Model *model(rknn_context context) {return reinterpret_cast<Model*>(context);}
std::vector<uint8_t> last_detector_input,last_classifier_input;
}
extern "C" {
const uint8_t *adas_test_input(int detector,size_t *size) {
  const auto &value=detector ? last_detector_input:last_classifier_input;
  *size=value.size();return value.data();
}
int rknn_init(rknn_context *context,void *,uint32_t size,uint32_t,rknn_init_extend *) {
  auto *m=new Model;m->detector=size>4000000;m->output.resize(m->detector ? 9:1);
  *context=reinterpret_cast<rknn_context>(m);return 0;
}
int rknn_destroy(rknn_context context) {delete model(context);return 0;}
int rknn_query(rknn_context context,rknn_query_cmd command,void *info,uint32_t) {
  const auto *m=model(context);
  if(command==RKNN_QUERY_IN_OUT_NUM) {auto *n=static_cast<rknn_input_output_num*>(info);n->n_input=1;n->n_output=m->detector ? 9:1;return 0;}
  auto *a=static_cast<rknn_tensor_attr*>(info);a->fmt=RKNN_TENSOR_NCHW;a->type=RKNN_TENSOR_FLOAT32;
  if(command==RKNN_QUERY_INPUT_ATTR) {
    a->n_dims=4;a->dims[0]=1;a->dims[1]=3;a->dims[2]=m->detector ? 384:128;a->dims[3]=m->detector ? 640:128;
    a->n_elems=a->dims[1]*a->dims[2]*a->dims[3];return 0;
  }
  if(command==RKNN_QUERY_OUTPUT_ATTR) {
    if(!m->detector) {a->n_dims=2;a->dims[0]=1;a->dims[1]=8;a->n_elems=8;return 0;}
    const int index=a->index;a->n_dims=4;a->dims[0]=1;a->dims[1]=index%3==0 ? 64:index%3==1 ? 3:1;
    a->dims[2]=48>>(index/3);a->dims[3]=80>>(index/3);a->n_elems=a->dims[1]*a->dims[2]*a->dims[3];return 0;
  }
  return -1;
}
int rknn_inputs_set(rknn_context context,uint32_t count,rknn_input input[]) {
  if(count!=1 || input[0].type!=RKNN_TENSOR_UINT8 || input[0].fmt!=RKNN_TENSOR_NHWC || input[0].pass_through!=0) return -1;
  const auto *bytes=static_cast<const uint8_t*>(input[0].buf);
  auto &value=model(context)->detector ? last_detector_input:last_classifier_input;
  value.assign(bytes,bytes+input[0].size);return 0;
}
int rknn_run(rknn_context context,rknn_run_extend *) {
  const char *delay=std::getenv("ADAS_TEST_NPU_MS");
  const int ms=delay ? std::atoi(delay):0;
  if(ms>0 && model(context)->detector) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  return 0;
}
int rknn_outputs_get(rknn_context context,uint32_t count,rknn_output output[],rknn_output_extend *) {
  auto *m=model(context);if(count!=m->output.size()) return -1;
  for(unsigned n=0;n<count;++n) {
    auto &v=m->output[n];
    if(!m->detector) {v.assign(8,0);v[4]=.95f;}
    else {
      const int h=48>>(n/3),w=80>>(n/3),cells=h*w,c=n%3==0 ? 64:n%3==1 ? 3:1;
      v.assign(c*cells,n%3==0 ? -80.f:0.f);
      if(n==0) for(int side=0;side<4;++side) v[(side*16+2)*cells+12*w+25]=80;
      if(n==1) v[2*cells+12*w+25]=.9f;
    }
    output[n].buf=v.data();output[n].size=v.size()*sizeof(float);
  }
  return 0;
}
int rknn_outputs_release(rknn_context,uint32_t,rknn_output[]) {return 0;}
}
