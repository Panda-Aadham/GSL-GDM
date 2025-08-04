#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

// TensorFlow Lite headers
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"

class SimpleTFLiteTest {
private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    
    // Air quality class labels
    std::vector<std::string> class_labels = {
        "Good",
        "Moderate", 
        "Unhealthy for Sensitive Groups",
        "Unhealthy",
        "Very Unhealthy",
        "Hazardous"
    };

public:
    bool loadModel(const std::string& model_path) {
        std::cout << "Loading model: " << model_path << std::endl;
        
        // Load the model
        model = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
        if (!model) {
            std::cerr << "Failed to load model!" << std::endl;
            return false;
        }
        
        // Build interpreter
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model, resolver);
        builder(&interpreter);
        
        if (!interpreter) {
            std::cerr << "Failed to create interpreter!" << std::endl;
            return false;
        }
        
        // Allocate tensors
        if (interpreter->AllocateTensors() != kTfLiteOk) {
            std::cerr << "Failed to allocate tensors!" << std::endl;
            return false;
        }
        
        std::cout << "Model loaded successfully!" << std::endl;
        printModelInfo();
        return true;
    }
    
    void printModelInfo() {
        // Input info
        int input = interpreter->inputs()[0];
        TfLiteTensor* input_tensor = interpreter->tensor(input);
        std::cout << "Input shape: [";
        for (int i = 0; i < input_tensor->dims->size; ++i) {
            std::cout << input_tensor->dims->data[i];
            if (i < input_tensor->dims->size - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        // Output info
        int output = interpreter->outputs()[0];
        TfLiteTensor* output_tensor = interpreter->tensor(output);
        std::cout << "Output shape: [";
        for (int i = 0; i < output_tensor->dims->size; ++i) {
            std::cout << output_tensor->dims->data[i];
            if (i < output_tensor->dims->size - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    void testPrediction() {
        // Get input tensor
        int input = interpreter->inputs()[0];
        TfLiteTensor* input_tensor = interpreter->tensor(input);
        int input_size = input_tensor->dims->data[1]; // Assuming shape [1, features]
        
        std::cout << "\nTesting with sample data..." << std::endl;
        std::cout << "Input features: " << input_size << std::endl;
        
        // Fill with sample normalized data (matching your Python preprocessing)
        float* input_data = interpreter->typed_input_tensor<float>(0);
        
        // Sample air quality data (normalized values from your dataset)
        std::vector<float> sample_data = {
            -0.74, -1.36, 0.0,      // hour, day_of_week, month
            0.0, 0.0, -0.89,        // aqi_value_no2, aqi_value_o3, aqi_value_pm25
            -0.31, -0.32, -0.88,    // concentration_no2, concentration_o3, concentration_pm25
            -0.89, 0.0,             // overall_aqi, season_encoded
            -0.89, -0.30, 0.45,     // aqi_max, aqi_mean, aqi_std
            -0.88, -0.50, 0.32,     // conc_max, conc_mean, conc_std
            0.65, 0.64, 0.03,       // no2_to_pm25_ratio, o3_to_pm25_ratio, no2_to_o3_ratio
            0.0, 0.0,               // hour_season_interaction, day_season_interaction
            0.89, 0.89, 0.0         // aqi_no2_deviation, aqi_o3_deviation, aqi_pm25_deviation
        };
        
        // Copy data to input tensor
        for (int i = 0; i < std::min(input_size, (int)sample_data.size()); ++i) {
            input_data[i] = sample_data[i];
        }
        
        // Run inference
        if (interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "Failed to run inference!" << std::endl;
            return;
        }
        
        // Get output
        float* output_data = interpreter->typed_output_tensor<float>(0);
        int output_size = interpreter->tensor(interpreter->outputs()[0])->dims->data[1];
        
        std::cout << "\nPrediction results:" << std::endl;
        
        // Find max probability and class
        float max_prob = output_data[0];
        int max_class = 0;
        
        for (int i = 0; i < output_size; ++i) {
            std::cout << "  Class " << i;
            if (i < class_labels.size()) {
                std::cout << " (" << class_labels[i] << ")";
            }
            std::cout << ": " << std::fixed << std::setprecision(4) << output_data[i] << std::endl;
            
            if (output_data[i] > max_prob) {
                max_prob = output_data[i];
                max_class = i;
            }
        }
        
        std::cout << "\nPredicted class: " << max_class;
        if (max_class < class_labels.size()) {
            std::cout << " (" << class_labels[max_class] << ")";
        }
        std::cout << std::endl;
        std::cout << "Confidence: " << std::fixed << std::setprecision(4) << max_prob << std::endl;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "==================================" << std::endl;
    std::cout << "TensorFlow Lite Model Test (WSL2)" << std::endl;
    std::cout << "==================================" << std::endl;
    
    std::string model_path = "air_quality_model.tflite";
    
    // Check command line argument
    if (argc > 1) {
        model_path = argv[1];
    }
    
    SimpleTFLiteTest test;
    
    if (!test.loadModel(model_path)) {
        std::cerr << "Failed to load model: " << model_path << std::endl;
        return 1;
    }
    
    test.testPrediction();
    
    std::cout << "\nTest completed!" << std::endl;
    return 0;
}