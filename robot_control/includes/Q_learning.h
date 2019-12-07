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

    void episode(int initialState); // runs chooseAnAction, until all states was visited
    void chooseAnAction(); // choose an action, either the best known action from Q, or by exploring randomly
    int getRandomAction(); // in case of exploring
    int maximum(int state, bool returnIndexOnly); // returns best reward when returnIndexOnly=false or index of this reward when returnIndexOnly=true
    int reward(int action); //  returns best reward with maximum function using returnIndexOnly=false

    void update_states_visited(int s);
    void update_reward_table(vector<int> s_v , int c_s );
    void pick_Q_table(vector<int> s_v , int c_s);
    void save_Q_table(vector<int> s_v , int c_s);
//////////////////////////
    void generate_all_binary_strings(vector<int> temp_vec, int i);
    int find_index(vector<int> vec_to_find);
//////////////////////////
    void restart();
    void print_Q();
    void print_all_Qs();
    void print_Q_percentage();
    void double_print();
    void clean_up();
    void run_tests();
    void load_Q_with(int load);
    void print_runs_vector();
    void print_latest_path();
    void print_path();

    void update_epsilon();

    void save_data();

    void set_gamma(double ga);
    void set_leaning_rate(double lr);
    void set_epsilon(double en);
//    int get_rand(const int& a, const int& b);
    int get_rand_small();
    int get_rand_big();

    ~Q_learning();

private:
    int states;
    int states_squared;
    int states_quad;
    int states_squared_doubled;
    int actions;
    double leaning_rate=0.1;
    double gamma = 0.7; //0.8
    int epsilon_times_ten=30;
    int iterations;
    int initialState = 0;
    int current_state;
    int runs_for_test=0;
    int epsilon_var=1;


    vector<int> runs_vector;

    vector<int> goal_states;
    vector<int> states_visited;
///
    vector<int> truth_table_row_TWO;
    vector<vector<int>> s_v_index;
    vector<int> vector_of_chosen_actions;
    vector<int> vector_of_chosen_actions_copy;
///    creates a vector of n1 vector<vector<int>> of n2 vector<int> containg n3 ints
///    vector<vector<vector<int>>> vec_3d(n1, vector<vector<int>>(n2, vector<int>(n3,0)));
    vector<vector<vector<int>>> vec_3d_Q;
///
    int newState;
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
//            {{-1, -1, -1, -1, 100, -1,-1},
//             {-1, -1, -1, 0, -1, 100,-1},
//             {-1, -1, -1, 100, -1, -1,-1},
//             {-1, 100, 0, -1, 100, -1,-1},
//             {-1, 0, 100, -1, 0, -1, 100},
//             {0, -1, -1, 0, -1, 100,-1},
//             {-1, 0, -1, -1, 0, 100,-1}};

//            {{-1,100, -1, -1, -1, 0, -1,-1, -1, -1},
//             {-1, 100, -1, 0, 0, 0,-1, -1, -1},
//             {-1,0, -1, 100, 0, 0, 0,-1, 0, -1},
//             {-1,-1, 0, 0, 100, -1, -1,-1, -1, -1},
//             {-1,-1, 0, 0, -1, 100, -1,-1, -1, -1},
//             {0,-1, 0, 0, -1, 0, 100,-1, -1, -1},
//             {-1,-1, 0, 0, -1, -1, -1,100, -1, -1},
//             {-1,0, 0, 0, -1, -1, -1, -1, 100, -1},
//             {0,0, -1, 0, -1, 0,0,-1, -1, 100},
//             {-1, 0, -1, 100, 0, 0, 0, 100, 100, -1}};


};