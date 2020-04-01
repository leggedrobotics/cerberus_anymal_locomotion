//
// Created by jhwangbo on 17.04.17.
//

#ifndef RAI_DLCTRL_SIMPLEMLP_HPP
#define RAI_DLCTRL_SIMPLEMLP_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include "iostream"
#include <fstream>
#include <cmath>

namespace RAI {

template<int StateDim, int ActionDim>
class MLP_fullyconnected {

 public:
  MLP_fullyconnected(const std::vector<int>& hiddensizes) 
  // :    act_(activation) 
  {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

    layersizes.push_back(StateDim);
    layersizes.reserve(layersizes.size() + hiddensizes.size());
    layersizes.insert(layersizes.end(), hiddensizes.begin(), hiddensizes.end());
    layersizes.push_back(ActionDim);
    ///[input hidden output]

    params.resize(2 * (layersizes.size() - 1));
    Ws.resize(layersizes.size() - 1);
    bs.resize(layersizes.size() - 1);
    lo.resize(layersizes.size());
  }

  bool readPolicyFromFile(const std::string& fileName) {
    std::stringstream parameterFileName;
    std::ifstream indata;
    indata.open(fileName);
    std::string line;
    getline(indata, line);
    std::stringstream lineStream(line);
    std::string cell;

    ///assign parameters
    for (int i = 0; i < params.size(); i++) {
      int paramSize = 0;

      if (i % 2 == 0) ///W resize
      {
        Ws[i / 2].resize(layersizes[i / 2 + 1], layersizes[i / 2]);
        params[i].resize(layersizes[i / 2] * layersizes[i / 2 + 1]);
      }
      if (i % 2 == 1) ///b resize
      {
        bs[(i - 1) / 2].resize(layersizes[(i + 1) / 2]);
        params[i].resize(layersizes[(i + 1) / 2]);
      }

      while (std::getline(lineStream, cell, ',')) { ///Read param
        params[i](paramSize++) = std::stof(cell);
        if (paramSize == params[i].size()) break;
      }
      if (i % 2 == 0) ///W copy
        memcpy(Ws[i / 2].data(), params[i].data(), sizeof(double) * Ws[i / 2].size());
      if (i % 2 == 1) ///b copy
        memcpy(bs[(i - 1) / 2].data(), params[i].data(), sizeof(double) * bs[(i - 1) / 2].size());
    }

    return true;
  }


  Eigen::VectorXd forward(Eigen::VectorXd state) {
    lo[0] = state;
    for (int cnt = 0; cnt < Ws.size() - 1; cnt++) {
      lo[cnt + 1] = Ws[cnt] * lo[cnt] + bs[cnt];

      for (int i = 0; i < lo[cnt + 1].size(); i++) {
        // if (act_.compare("tanh") == 0) {
          lo[cnt + 1][i] = std::tanh(lo[cnt + 1][i]);
        // }
        // else if (act_.compare("relu") == 0) {
          // if (lo[cnt + 1][i] < 0) lo[cnt + 1][i] = 0;
        // }
        // else if (act_.compare("softsign") == 0) {
          // lo[cnt + 1][i] =  lo[cnt + 1][i] / (std::abs( lo[cnt + 1][i]) + 1.0);
        // }
      }
    }

    lo[lo.size() - 1] = Ws[Ws.size() - 1] * lo[lo.size() - 2] + bs[bs.size() - 1]; /// output layer

    return lo[lo.size() - 1];
  }
 private:

//Eigen::MatrixXd output_;
  std::vector<Eigen::VectorXd> params;
  std::vector<Eigen::MatrixXd> Ws;
  std::vector<Eigen::VectorXd> bs;
  std::vector<Eigen::VectorXd> lo;

  std::vector<int> layersizes;
};

}


#endif //RAI_SIMPLEMLP_HPP 
