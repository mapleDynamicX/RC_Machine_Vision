#include<iostream>
#include<opencv2/opencv.hpp>
#include <chrono>

using namespace std;
using namespace cv;

struct Bbox
{
    Bbox()
    {
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;
        score = 0;
        class_probability = 0;
    }
    //四个点的坐标
    float x1; 
    float y1;
    float x2;
    float y2;
    float score; //是物体的概率
    float class_probability; //是这个类的的概率，因为是单类别这个参数作用不大
};

void coordinate_convert(float* pdata, vector<Bbox>& bboxes)
{
    Bbox bbox;
    float x, y, w, h;
    x = pdata[0];
    y = pdata[1];
    w = pdata[2];
    h = pdata[3];
    bbox.x1 = x - w / 2;
    bbox.y1 = y - h / 2;
    bbox.x2 = x + w / 2;
    bbox.y2 = y + h / 2;
    bbox.score = pdata[4];
    bbox.class_probability = pdata[5];
    bboxes.push_back(bbox);
}

float iou(const Bbox& box1, const Bbox& box2)
{
    float area1 = (box1.x2 - box1.x1) * (box1.y2 - box1.y1);
    float area2 = (box2.x2 - box2.x1) * (box2.y2 - box2.y1);

    float x1 = max(box1.x1, box2.x1);
    float y1 = max(box1.y1, box2.y1);
    float x2 = min(box1.x2, box2.x2);
    float y2 = min(box1.y2, box2.y2);

    float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float union_area = area1 + area2 - intersection;
    return intersection / union_area;
}

vector<Bbox> nms(const vector<Bbox>& bboxes, float iouThreshold = 0.45)
{
    vector<Bbox> result;
    if (bboxes.empty()) return result;

    vector<pair<int, float>> scores;
    for (size_t i = 0; i < bboxes.size(); ++i) {
        scores.emplace_back(i, bboxes[i].score);
    }
    sort(scores.begin(), scores.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
        });

    //vector<bool> keep(bboxes.size(), true);

    //for (size_t i = 0; i < scores.size(); ++i) {
    //    int index = scores[i].first;
    //    if (!keep[index]) continue;

    //    result.push_back(bboxes[index]);

    //    for (size_t j = i + 1; j < scores.size(); ++j) {
    //        int nextIndex = scores[j].first;
    //        if (keep[nextIndex]) {
    //            float iouValue = iou(bboxes[index], bboxes[nextIndex]);
    //            if (iouValue > iouThreshold) {
    //                keep[nextIndex] = false;
    //            }
    //        }
    //    }
    //}
    int index = scores[0].first;
    result.push_back(bboxes[index]);
    return result;
}

vector<Bbox> postprocess(const Mat& output, float confThreshold = 0.25)
{
    vector<Bbox> bboxes;
    vector<Bbox> result;
    float* pdata = (float*)output.data; 
    int length = 6;

    for (int i = 0; i < output.total() / length; i++)
    {
        double confidence = pdata[4];
        if (confidence > confThreshold)
        {
            coordinate_convert(pdata, bboxes);
        }
        pdata += length;
    }

    result = nms(bboxes, 0.25);

    return result;
}

void drawBoxes(Mat& img, const vector<Bbox>& bboxes)
{
    for (const auto& box : bboxes)
    {
        rectangle(img, Point(box.x1, box.y1), Point(box.x2, box.y2), Scalar(0, 255, 0), 1);
        //putText(img, "basket", Point(box.x1, box.y1 - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    }
}

cv::Mat shapeimg(cv::Mat img)
{

    cv::Mat resize_img;
    cv::resize(img, resize_img, cv::Size(640, 360));
    cv::Mat padding_img;

    cv::copyMakeBorder(resize_img, padding_img, 140, 140, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return padding_img;
}

// int main()
// {

//     dnn::Net net = dnn::readNetFromONNX("D:\\software\\opencv++\\opencv\\code\\test\\test\\best.onnx");

//     VideoCapture cap("D:\\software\\opencv++\\opencv\\code\\test\\test\\b97206737226538b69fedde23f562aad_raw.mp4");

//     while (true)
//     {
//         auto start = std::chrono::steady_clock::now();

//         Mat img;
//         if (!cap.read(img))
//         {
//             break;
//         }
//         img = shapeimg(img);
//         Mat blob = dnn::blobFromImage(img, 1.0 / 255.0, Size(640, 640), Scalar(0, 0, 0), true);
//         net.setInput(blob);
//         vector<Mat> outputs;
//         vector<String> outNames = { "output0" };
//         net.forward(outputs, outNames);
//         cv::Mat output = outputs[0];
//         vector<Bbox> currentBoxes = postprocess(output, 0.25);
//         drawBoxes(img, currentBoxes);
//         //imshow("img", img);
//         //waitKey(1);
//         auto end = std::chrono::steady_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         std::cout <<"time:" << duration.count() <<std::endl;
//     }
// }



//接口
/*
    @brief yolov5模型
    @param img 输入图片
    @param bbox 自定义的类对象，详细请看文件最上方, 
    @param net 加载好onnx的神经网络（yolov5训练出来单类别神经网络模型）
    @return bool, 没识别到返回false, 识别到返回true且坐标放在传参的bbox中

*/
bool neural_network(cv::Mat& img, Bbox& bbox, dnn::Net& net)
{
    img = shapeimg(img);
    Mat blob = dnn::blobFromImage(img, 1.0 / 255.0, Size(640, 640), Scalar(0, 0, 0), true);
    net.setInput(blob);
    vector<Mat> outputs;
    vector<String> outNames = { "output0" };
    net.forward(outputs, outNames);
    cv::Mat output = outputs[0];
    vector<Bbox> currentBoxes = postprocess(output, 0.25);
    //drawBoxes(img, currentBoxes);
    
    if(currentBoxes.size() == 0)
    {
        return false;
    }
    else
    {
        bbox = currentBoxes[0];
        return true;
    }
}