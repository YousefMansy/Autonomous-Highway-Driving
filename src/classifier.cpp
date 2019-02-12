#include "classifier.h"
#include <math.h>
#include <string>
#include <vector>

using Eigen::ArrayXd;
using std::string;
using std::vector;

// Initializes GNB
GNB::GNB()
{
    /**
   * TODO: Initialize GNB, if necessary. May depend on your implementation.
   */

    left_means = ArrayXd(4);
    keep_means = ArrayXd(4);
    right_means = ArrayXd(4);

    left_stds = ArrayXd(4);
    keep_stds = ArrayXd(4);
    right_stds = ArrayXd(4);
}

GNB::~GNB() {}

void GNB::train(string data_path, string labels_path)
{
    /**
   * Trains the classifier with N data points and labels.
   * @param data - array of N observations
   *   - Each observation is a tuple with 4 values: s, d, s_dot and d_dot.
   *   - Example : [[3.5, 0.1, 5.9, -0.02],
   *                [8.0, -0.3, 3.0, 2.2],
   *                 ...
   *                ]
   * @param labels - array of N labels
   *   - Each label is one of "left", "keep", or "right".
   *
   */

    // Load data from text_file
    vector<vector<double>> data = Load_State(labels_path);
    vector<string> labels = Load_Label(data_path);

    // Initializations
    ArrayXd left_sums = ArrayXd(4);
    left_sums << 0, 0, 0, 0;
    float left_size = 0;

    ArrayXd keep_sums = ArrayXd(4);
    keep_sums << 0, 0, 0, 0;
    float keep_size = 0;

    ArrayXd right_sums = ArrayXd(4);
    right_sums << 0, 0, 0, 0;
    float right_size = 0;

    for (int i = 0; i < labels.size(); i++)
    {
        if (labels[i] == "left")
        {
            left_sums += ArrayXd::Map(data[i].data(), data[i].size());
            left_size++;
        }
        if (labels[i] == "keep")
        {
            keep_sums += ArrayXd::Map(data[i].data(), data[i].size());
            keep_size++;
        }
        if (labels[i] == "right")
        {
            right_sums += ArrayXd::Map(data[i].data(), data[i].size());
            right_size++;
        }
    }

    left_means = left_sums / left_size;
    keep_means = keep_sums / keep_size;
    right_means = right_sums / right_size;

    ArrayXd left_stds_numerator = ArrayXd(4);
    left_stds_numerator << 0, 0, 0, 0;

    ArrayXd keep_stds_numerator = ArrayXd(4);
    keep_stds_numerator << 0, 0, 0, 0;

    ArrayXd right_stds_numerator = ArrayXd(4);
    right_stds_numerator << 0, 0, 0, 0;

    for (int i = 0; i < labels.size(); ++i)
    {
        ArrayXd data_point = ArrayXd::Map(data[i].data(), data[i].size());
        if (labels[i] == "left")
        {
            left_stds_numerator += (data_point - left_means) * (data_point - left_means);
        }
        if (labels[i] == "keep")
        {
            keep_stds_numerator += (data_point - keep_means) * (data_point - keep_means);
        }
        if (labels[i] == "right")
        {
            right_stds_numerator += (data_point - right_means) * (data_point - right_means);
        }
    }

    left_stds = (left_stds_numerator / left_size).sqrt();
    keep_stds = (keep_stds_numerator / keep_size).sqrt();
    right_stds = (right_stds_numerator / right_size).sqrt();

    left_prior = left_size / labels.size();
    keep_prior = keep_size / labels.size();
    right_prior = right_size / labels.size();
}

string GNB::predict(const vector<double> &sample)
{
    /**
   * Once trained, this method is called and expected to return 
   *   a predicted behavior for the given observation.
   * @param observation - a 4 tuple with s, d, s_dot, d_dot.
   *   - Example: [3.5, 0.1, 8.5, -0.2]
   * @output A label representing the best guess of the classifier. Can
   *   be one of "left", "keep" or "right".
   *
   * TODO: Complete this function to return your classifier's prediction
   */
    double left_p = 1.0;
    double keep_p = 1.0;
    double right_p = 1.0;
    for (int i = 0; i < 4; ++i)
    {
        left_p *= (1.0 / sqrt(2.0 * M_PI * pow(left_stds[i], 2))) * exp(-0.5 * pow(sample[i] - left_means[i], 2) / pow(left_stds[i], 2));
        keep_p *= (1.0 / sqrt(2.0 * M_PI * pow(keep_stds[i], 2))) * exp(-0.5 * pow(sample[i] - keep_means[i], 2) / pow(keep_stds[i], 2));
        right_p *= (1.0 / sqrt(2.0 * M_PI * pow(right_stds[i], 2))) * exp(-0.5 * pow(sample[i] - right_means[i], 2) / pow(right_stds[i], 2));
    }
    left_p *= left_prior;
    keep_p *= keep_prior;
    right_p *= right_prior;

    double probs[3] = {left_p, keep_p, right_p};
    double max = left_p;
    double max_index = 0;

    for (int i = 1; i < 3; ++i)
    {
        if (probs[i] > max)
        {
            max = probs[i];
            max_index = i;
        }
    }
    return this->possible_labels[max_index];
}

// Load state from .txt file
vector<vector<double>> Load_State(string file_name)
{
    ifstream in_state_(file_name.c_str(), ifstream::in);
    vector<vector<double>> state_out;
    string line;

    while (getline(in_state_, line))
    {
        std::istringstream iss(line);
        vector<double> x_coord;

        string token;
        while (getline(iss, token, ','))
        {
            x_coord.push_back(stod(token));
        }
        state_out.push_back(x_coord);
    }

    return state_out;
}

// Load labels from .txt file
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector<string> label_out;
    string line;
    while (getline(in_label_, line))
    {
        std::istringstream iss(line);
        string label;
        iss >> label;

        label_out.push_back(label);
    }

    return label_out;
}