#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using Eigen::ArrayXd;
using std::string;
using std::vector;

class GNB {
 public:
  /**
   * Constructor
   */
  GNB();

  /**
   * Destructor
   */
  virtual ~GNB();

  /**
   * Train classifier
   */
  void train(string data_path, string labels_path);

  /**
   * Predict with trained classifier
   */
  string predict(const vector<double> &sample);

  vector<vector<double>> Load_State(string file_name);
  vector<string> Load_Label(string file_name);


  vector<string> possible_labels = {"left","keep","right"};
  
  ArrayXd left_means;
  ArrayXd left_stds;
  double left_prior;
  
  ArrayXd keep_means;
  ArrayXd keep_stds;
  double keep_prior;
  
  ArrayXd right_means;
  ArrayXd right_stds;
  double right_prior;
};

#endif  // CLASSIFIER_H