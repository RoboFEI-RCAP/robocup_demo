#include "booster_vision/model/trt/impl.h"
#include "booster_vision/model/trt/logging.h"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdexcept>

Logger logger;

void YoloV8DetectorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }
    
    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] * model_input_dims_.d[3];
    output_size_ = model_output_dims_.d[0] * model_output_dims_.d[1] * model_output_dims_.d[2];
    input_buff_ = (float*)malloc(input_size_ * sizeof(float));
    output_buff_ = (float*)malloc(output_size_ * sizeof(float));
    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&output_mem_, output_size_ * sizeof(float));
    if (async_infer_)
    {
        cudaStreamCreate(&stream_);
    }
    else
    {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }


    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1), output_mem_);
  
    std::cout << "det model initialization, done!"  << std::endl;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
    std::vector<float> factors;
    if (!PreProcess(img, factors))
    {
        return {};
    }

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_,input_buff_, input_size_ * sizeof(float),cudaMemcpyHostToDevice, async_infer_);

    bool status = false;
    if (async_infer_)
    {
        status = context_->enqueueV3(stream_);
    }
    else
    {
        status = context_->executeV2(bindings_.data());
    }
    
    if (!status)
    {
        return {};
    }

    // Memcpy from device output buffers to host output buffers
    MemcpyBuffers(output_buff_, output_mem_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,async_infer_);

    if (async_infer_)
    {
        cudaStreamSynchronize(stream_);
    }

    img_width_ = img.cols;
    img_height_ = img.rows;
    std::vector<booster_vision::DetectionRes> outputs = PostProcess(factors);

    std::cout << "finish inference " << std::endl;
    return outputs;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
  cudaStreamDestroy(stream_);
  cudaFree(input_mem_);
  cudaFree(output_mem_);
  free(input_buff_);
  free(output_buff_);
}


bool YoloV8DetectorTRT::LoadEngine()
{
    std::ifstream input(model_path_, std::ios::binary);
    if (!input)
    {
        return false;
    }
    input.seekg(0, input.end);
    const size_t fsize = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> bytes(fsize);
    input.read(bytes.data(), fsize);

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(bytes.data(), bytes.size()), InferDeleter());
    if (!engine_)
        return false;
    
    int nbio = engine_->getNbIOTensors();
    const char* inputname = engine_->getIOTensorName(0);
    const char* outputname = engine_->getIOTensorName(engine_->getNbIOTensors() - 1);
    Dims input_shape = engine_->getTensorShape(inputname);
    Dims output_shape = engine_->getTensorShape(outputname);
    model_input_dims_ = Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output_dims_ = Dims4(output_shape.d[0], output_shape.d[1], output_shape.d[2], output_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " " << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model output dims: " << output_shape.d[0] << " " << output_shape.d[1] << " " << output_shape.d[2] << std::endl;
   

    return true;
}

bool YoloV8DetectorTRT::PreProcess(const cv::Mat& img, std::vector<float>& factors)
{
    cv::Mat mat;
    int rh = img.rows;
    int rw = img.cols;
    int rc = img.channels();
    
    cv::cvtColor(img, mat, cv::COLOR_BGR2RGB);
    // mat = img;
    int maxImageLength = rw > rh ? rw : rh;
    if (squre_input_ && (model_input_dims_.d[2] == model_input_dims_.d[3]))
    {
        factors.emplace_back(maxImageLength / 640.0);
        factors.emplace_back(maxImageLength / 640.0);
    }
    else
    {
        factors.emplace_back(img.rows / 640.0);
        factors.emplace_back(img.cols / 640.0);
    }
    cv::Mat maxImage = cv::Mat::zeros(maxImageLength, maxImageLength, CV_8UC3);
    maxImage = maxImage * 255;
    cv::Rect roi(0, 0, rw, rh);
    mat.copyTo(cv::Mat(maxImage, roi));
    cv::Mat resizeImg;
    int length = 640;
    cv::resize(maxImage, resizeImg, cv::Size(length, length), 0.0f, 0.0f, cv::INTER_LINEAR);
    resizeImg.convertTo(resizeImg, CV_32FC3, 1 / 255.0);
    rh = resizeImg.rows;
    rw = resizeImg.cols;
    rc = resizeImg.channels();
    
    for (int i = 0; i < rc; ++i) {
        cv::extractChannel(resizeImg, cv::Mat(rh, rw, CV_32FC1, input_buff_ + i * rh * rw), i);
    }
    return true;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::PostProcess(std::vector<float> factors)
{
    const int outputSize = model_output_dims_.d[1];
    //float* output = static_cast<float*>(output_buff_);
    cv::Mat outputs(outputSize, 8400, CV_32F, output_buff_);

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Preprocessing output results
    const int class_num = outputSize - 4; // 4 for box[x,y,w,h]
    int rows = outputs.size[0];
    int dimensions = outputs.size[1];
    bool yolov8 = false;

    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs.size[1];
        dimensions = outputs.size[0];

        outputs = outputs.reshape(1, dimensions);
        cv::transpose(outputs, outputs);
    }

    float* data = (float*)outputs.data;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num, CV_32FC1, classes_scores);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > confidence_)
        {
            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * factors[0]);
            int top = int((y - 0.5 * h) * factors[1]);

            int width = int(w * factors[0]);
            int height = int(h * factors[1]);

            if (left < 0 || left > img_width_ - 1) continue;
            if (top < 0 || top > img_height_ - 1) continue;

            int right = std::min(img_width_ - 1, left + width);
            int bottom = std::min(img_height_ - 1, top + height);
            width = right - left;
            height = bottom - top;
            if (width < 3 || height < 3) continue;

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);
        }

        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.4, nms_result);

    std::vector<booster_vision::DetectionRes> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        booster_vision::DetectionRes result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}

void YoloV8DetectorTRT::MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize, cudaMemcpyKind memcpyType, bool const async)
{
    if (async) {
        cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream_);
    } else {
        cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType);
    }
}

