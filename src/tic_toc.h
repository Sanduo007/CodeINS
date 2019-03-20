/*
 * @Author: yibin wu 
 * @Date: 2019-02-26 15:00:32 
 * @Last Modified by: yibin wu
 * @Last Modified time: 2019-02-26 15:09:43
 */

#ifndef TIC_TOC_H_
#define TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000; //return ms
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif /* TIC_TOC_H_ */
