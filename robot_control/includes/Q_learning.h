#include <iostream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <algorithm>
#include <math.h>
#include <string>
#include <exception>
#include <fstream>
#include <iterator>
#include <random>

#define MARBLE_ 100 ///Marble
#define ILLEGAL -1 /// Wall
#define A_PATH 0 /// Path

#define NODE_AMOUNT 16

using namespace std;

class Q_learning
{
public:
    Q_learning(int number_of_states, int i);
    void learning_trials(); // runs the desired amount "i" of episodes
    void episode(int initialState); // runs chooseAnAction, until all states is visited
    void chooseAnAction(); // choose an action, either the best known action from Q, or by exploring randomly
    int getRandomAction(); // in case of exploring
    int maximum(int state, bool returnIndexOnly); // returns best reward when returnIndexOnly=false or index of this reward when returnIndexOnly=true
    int reward(int action); //  returns best reward with maximum function using returnIndexOnly=false

    void update_states_visited(int s);
    void update_reward_table(vector<int> s_v , int c_s );
    void pick_Q_table(vector<int> s_v , int c_s);
    void save_Q_table(vector<int> s_v , int c_s);

    int find_index(vector<int> vec_to_find);

    void restart();
    void print_Q();
    void clean_up();
    void print_runs_vector();
    void print_latest_path();
    void update_epsilon();
    void save_data();
    void set_gamma(double ga);
    void set_leaning_rate(double lr);
    void set_epsilon(double en);
    int get_rand_small();
    int get_rand_big();

    ~Q_learning();

private:
    int states;
    int actions;
    double leaning_rate=0.1;
    double gamma = 0.9;
    int epsilon_times_ten=20;
    int iterations;
    int initialState = 0;
    int current_state;
    int runs_for_test=0;
    int epsilon_var=1;

    vector<int> runs_vector;
    vector<int> goal_states;
    vector<int> states_visited;
    vector<vector<int>> s_v_index;
    vector<int> vector_of_chosen_actions;
    vector<int> vector_of_chosen_actions_copy;
    vector<vector<vector<int>>> vec_3d_Q;
    vector<int> Q; /// the one thats in use
    vector<vector<int>> reward_array;
    vector<vector<int>> reward_array0= /// starting point in state 0


#if NODE_AMOUNT==16
/// /// ACTIONS: 0      1       2       3       4       5       6       7       8       9       10      11      12      13      14      15      || STATES
            {{ILLEGAL,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 0
             {A_PATH,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 1
             {ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 2
             {A_PATH,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 3
             {ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 4
             {ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 5
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 6
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 7
             {A_PATH,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,MARBLE_,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 8
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 9
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 10
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,MARBLE_,ILLEGAL,ILLEGAL}, /// 11
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 12
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,MARBLE_,MARBLE_}, /// 13
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 14
             {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 15
            };
#endif

#if NODE_AMOUNT==8
/// ACTIONS: 0      1       2       3       4       5       6       7          || STATES
       {{ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 0
        {A_PATH,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL}, /// 1
        {ILLEGAL,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 2
        {ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,MARBLE_}, /// 3
        {ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 4
        {ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL}, /// 5
        {ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 6
        {ILLEGAL,ILLEGAL,ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL,ILLEGAL,ILLEGAL}, /// 7
    };
#endif

#if NODE_AMOUNT == 4
/////   ACTIONS: 0      1       2       3          || STATES
            {{ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 0
             {A_PATH,ILLEGAL,MARBLE_,MARBLE_}, /// 1
             {ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 2
             {ILLEGAL,MARBLE_,ILLEGAL,ILLEGAL}, /// 3
            };
#endif
};
