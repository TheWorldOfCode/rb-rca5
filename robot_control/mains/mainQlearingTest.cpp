#include <iostream>
#include <iomanip>
#include <ctime>

#include "Q_learning.h"

using namespace std;

void test1()
{
    Q_learning Q_LEARNING(16,10000);
    Q_LEARNING.set_leaning_rate(0.1);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();

}
void test2()
{
    Q_learning Q_LEARNING(16,5000);
    Q_LEARNING.set_leaning_rate(0.1);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}

void test3()
{
    Q_learning Q_LEARNING(16,2500);
    Q_LEARNING.set_leaning_rate(0.1);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}

void test4()
{
    Q_learning Q_LEARNING(16,2500);
    Q_LEARNING.set_leaning_rate(0.2);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}
void test5()
{
    Q_learning Q_LEARNING(16,2500);
    Q_LEARNING.set_leaning_rate(0.1);
    Q_LEARNING.set_gamma(0.8);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}

void test6()
{
    Q_learning Q_LEARNING(16,2500);
    Q_LEARNING.set_leaning_rate(0.1);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.3);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}

void test7()
{
    Q_learning Q_LEARNING(16,2000);
    Q_LEARNING.set_leaning_rate(0.2);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}


void test8()
{
    Q_learning Q_LEARNING(16,2000);
    Q_LEARNING.set_leaning_rate(0.3);
    Q_LEARNING.set_gamma(0.9);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}
void test9()
{
    Q_learning Q_LEARNING(16,2000);
    Q_LEARNING.set_leaning_rate(0.3);
    Q_LEARNING.set_gamma(0.8);
    Q_LEARNING.set_epsilon(0.2);
    Q_LEARNING.learning_trials();
    Q_LEARNING.print_runs_vector();
    Q_LEARNING.print_latest_path();
    Q_LEARNING.clean_up();
}


int main()
{
    test1();
    test2();
    test4();





    return 0;
}
