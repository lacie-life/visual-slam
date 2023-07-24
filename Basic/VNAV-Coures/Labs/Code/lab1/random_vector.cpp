#include <numeric>
#include <iostream>
#include <vector>

using namespace std;

class RandomVector{
  vector<double> vect;
  
  public:
    RandomVector(int size, double max_val = 1) {
        // TODO: Write your code here
        vect = std::vector<double>(size);
        for (auto i = 0; i < size; ++i) {
            vect[i] = rand() * max_val / RAND_MAX;
        }
    }

    void print(){
        // TODO: Write your code here
        for (auto i = 0; i < vect.size(); ++i) {
            std::cout << vect[i] << "  ";
        }
        std::cout << endl;
    }

    double mean(){
        // TODO: Write your code here
        if (vect.empty()) {
            return 0;
        }
        return std::accumulate(vect.begin(), vect.end(), 0.0)/vect.size();
    }

    double max(){
        // TODO: Write your code here
        double max_element = vect[0];
        for (auto k = 1; k < vect.size(); k++){
            if (vect[k] > max_element) {
                max_element = vect[k];
            }
        }
        return max_element;
    }

    double min(){
        //TODO:  Write your code here
        double min_element = vect[0];
        for (auto k = 1; k < vect.size(); k++){
            if (vect[k] < min_element) {
                min_element = vect[k];
            }
        }
        return min_element;
    }

    void printHistogram(int bins){
        double max = this->max();
        double min = this->min();
        double range = max-min;
        //std::cout << range << std::endl;
        double binToBin = range/bins;
        //std::cout << binToBin << std::endl;
        std::vector<double> hist(bins,0);
        for (auto i=0; i<vect.size(); i++){
            int bin=(int) ((vect[i]-min)/binToBin);
            //std::cout << bin << std::endl;
            hist[bin] ++;
        }
        for (auto i=0; i<bins; i++){
            cout << hist[i] << " ";
        }
        cout << endl; 
        // find the maximum in the historgram
        int maxHist =0;
        for (auto i=0; i<bins; i++){
            if (hist[i]>maxHist) {
                maxHist= hist[i];
            }
        }
        // drawing the histogram
        for (auto i=maxHist; i>0; i--){
            for (auto j=0; j<bins; j++){
                if (hist[j]>=i){
                    std::cout << "*** ";
                }else
                    std::cout << "    " ;
            }
            std::cout << std::endl;
        }
    }

};
