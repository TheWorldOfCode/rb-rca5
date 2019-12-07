#include <iostream>
#include <iomanip>
#include <ctime>

#include "../includes/Q_learning.h"

using namespace std;


void test1()
{
	/////////// Test 1
	Q_learning Q_LEARNING(16,10000);
	Q_LEARNING.set_leaning_rate(0.2);
	Q_LEARNING.set_gamma(0.9);
	Q_LEARNING.set_epsilon(0.2);
	Q_LEARNING.learning_trials();
	Q_LEARNING.print_runs_vector();
	Q_LEARNING.print_latest_path();
	Q_LEARNING.clean_up();

}
void test2()
{
	/////////// Test 2
	Q_learning Q_LEARNING(16,5000);
	Q_LEARNING.set_leaning_rate(0.2);
	Q_LEARNING.set_gamma(0.9);
	Q_LEARNING.set_epsilon(0.2);
	Q_LEARNING.learning_trials();
	Q_LEARNING.print_runs_vector();
	Q_LEARNING.print_latest_path();
	Q_LEARNING.clean_up();
}

void test3()
{
	/////////// Test 3
	Q_learning Q_LEARNING(16,1000);
	Q_LEARNING.set_leaning_rate(0.1);
	Q_LEARNING.set_gamma(0.9);
	Q_LEARNING.set_epsilon(0.2);
	Q_LEARNING.learning_trials();
	Q_LEARNING.print_runs_vector();
	Q_LEARNING.print_latest_path();
	Q_LEARNING.clean_up();
}



int main()
{
	test2();
	return 0;
}


