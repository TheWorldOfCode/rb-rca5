#include "Q_learning.h"
#define COUT 1
#define NEW_RANDOM 1
#define MASSIVE_DEBUG 0


Q_learning::Q_learning(int number_of_states, int i)
{
    cout << "Constructor: hello" << endl;
    vector<int> int_list_tmp;
    for (int i=1;i<=100;i++)
        {int_list_tmp.push_back(i);}
    states=number_of_states;
    actions=number_of_states;
    iterations=i;
    vector<int> Q_temp(states,0);
    Q=Q_temp;
#if NEW_RANDOM ==0
    srand(time(NULL));
#endif
    vector<int> tmp_goal(states,1);
    goal_states=tmp_goal;
    vector<int> tmp_visited(states,0);
    states_visited=tmp_visited;
    reward_array=reward_array0;
    update_states_visited(0);
    vector<vector<vector<int>>> vec_3d_temp(pow(2,states), vector<vector<int>>(states, vector<int>(states)));
    vec_3d_Q=vec_3d_temp;
    cout << "Constructor: goodbye" << endl;
}

Q_learning::~Q_learning()
{
    clean_up();

}

int Q_learning::reward(int action)///
{
    return reward_array[current_state][action] + (gamma * maximum(action, false));/////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void Q_learning::learning_trials()
{
    cout << "learning_trials(): hello " << endl;
    vector_of_chosen_actions.push_back(0);
    for(int j = 0; j <= (iterations - 1); j++)
        {
            {
                episode(initialState);

                if(j==iterations-1)
                    {
                    vector_of_chosen_actions_copy=vector_of_chosen_actions;
                    }
#if COUT
            cout << "The run through all states took " <<runs_for_test << " steps" <<endl;
           cout << "the last state was: " << current_state << endl;
            cout << "****************************************************************************************************************"<<endl << endl;

#endif
            runs_vector.push_back(runs_for_test);
            restart();
            epsilon_var++;
            } // i
        } // j
    cout << "learning_trials(): goodbye" << endl;
}

void Q_learning::episode(int initialState)
{
    current_state = initialState;
#if MASSIVE_DEBUG
cout << "entry to episodes current state is: " << current_state <<endl<<endl;
#endif
    do {
        chooseAnAction();

        } while(goal_states != states_visited );
#if MASSIVE_DEBUG
    cout << "exit from episodes current state is " << current_state <<endl<<endl;
#endif
#if COUT
    cout << "****************************************************************************************************************"<<endl;
    cout << "Test number: " << epsilon_var<< " out of "<< iterations<< endl;
    cout << epsilon_times_ten<<"% chance of exploring" << endl;
    cout << "The number of Q tables used was " << s_v_index.size() << endl;
#endif
}

void Q_learning::chooseAnAction()
{
    runs_for_test++;
    int possibleAction;
#if NEW_RANDOM
    int choice = get_rand_big();
#else

    const int rand_max_2 = 99;
    const int choice = rand() % rand_max_2;
#endif

#if MASSIVE_DEBUG
    cout << "at entry to chooseAnAction, current state is: " << current_state <<endl;

    cout << "and the vector states_visited contains: " <<endl;
    for(int i=0;i<states_visited.size();i++)
        {
        cout << states_visited[i] << " ";
        }
        cout << endl << "Q-table is loaded with:  index = find_index(states_visited)="<<find_index(states_visited)<<endl<<"from  Q=vec_3d_Q[index][current_state]" <<endl<< endl;
#endif
    pick_Q_table(states_visited,
                 current_state);
#if MASSIVE_DEBUG
   cout<< "current Q is" << endl;
    print_Q();
#endif
    update_epsilon();
    if (choice > (99 - epsilon_times_ten)) /// if choice is larger than 99-epsilon_times_ten, take random action
        {
        //Randomly choose a possible action connected to the current state.
        possibleAction = getRandomAction();
#if MASSIVE_DEBUG
        cout << "random action was chosen to: " << possibleAction <<endl<<endl;
#endif

        } else /// if choice is smaller than 99-epsilon_times_ten, take best action
        {

        possibleAction = maximum(current_state, true);
#if MASSIVE_DEBUG
        cout << "Best action was choosen to: " << possibleAction <<endl<<endl;
#endif
        }

    vector_of_chosen_actions.push_back(possibleAction);
    if (reward_array[current_state][possibleAction] >= 0)
        {
        update_reward_table(states_visited, current_state);
#if MASSIVE_DEBUG
        cout << "the chosen action was legal and the reward is saved at Q by action index: " << possibleAction <<endl<<endl;
#endif
        Q[possibleAction] = Q[possibleAction] + leaning_rate * ((reward(possibleAction)) -
                                                                Q[possibleAction]);
        const int old_state = current_state;
        current_state = possibleAction;
#if MASSIVE_DEBUG
        cout << "the old state was: " << old_state <<endl << "and now current state is CHANGED/MOVED to the legal possible action: "<< possibleAction << endl;

        cout << "the Q-table is stored by folowwing values:" << endl << "states_visited vector: ";
        for(int i=0;i<states_visited.size();i++)
            {
            cout << states_visited[i] << " ";
            }
        cout << endl << "and old state:" << old_state<<endl;
        cout << "by:  index = find_index(states_visited)="<<find_index(states_visited)<<endl<<"to  Q=vec_3d_Q[index][old_state]" <<endl<< endl;
#endif
        save_Q_table(states_visited, old_state);
        update_states_visited(current_state);
        }
    else
        {
#if MASSIVE_DEBUG
        cout << "the chosen action was NOT LEGAL"<< endl<< "and Q action is marked with -1 in update index " << possibleAction <<" of Q"<<endl;

    cout << "the Q-table is stored by folowwing values:" << endl << "states_visited vector: ";
    for(int i=0;i<states_visited.size();i++)
        {
        cout << states_visited[i] << " ";
        }
    cout << endl << "and current_state:" << current_state<<endl;
    cout << "by:  index = find_index(states_visited)="<<find_index(states_visited)<<endl<<"to  Q=vec_3d_Q[index][current_state]" <<endl<< endl;
#endif
        /// in case the posibleAction was unvalid, the state is marked as unreachable
        Q[possibleAction] = -1;
        save_Q_table(states_visited, current_state);
        }



}

int Q_learning::maximum(int state, bool returnIndexOnly)
{

    int winner = 0;
    bool foundNewWinner;
    bool done = false;


        foundNewWinner = false;
        for(int i = 0; i <= (states - 1); i++)
            {

            if( i != winner )
                {

                if(Q[i] > Q[winner])
                    {
                    winner = i;
                    foundNewWinner = true;
                    }
                }
            } // i

    if(returnIndexOnly == true)
        {
        return winner;
        }
    else
        {
        return Q[winner];
        }
}

int Q_learning::getRandomAction()
{
#if NEW_RANDOM
    int action_tmpl;
    bool choiceIsValid = false;
    //Randomly choose a possible action connected to the current state.
    do {

        action_tmpl = get_rand_small();
        if(reward_array[current_state][action_tmpl] > -1)
            {
            choiceIsValid = true;
            }
        } while(choiceIsValid == false);

    return action_tmpl;
#else
    int action;
    //int rand_max=6;
    bool choiceIsValid = false;
    //Randomly choose a possible action connected to the current state.
    do {


        action = rand() % actions;
        if(reward_array[current_state][action] > -1)
            {
            choiceIsValid = true;
            }
        } while(choiceIsValid == false);

    return action;
#endif

}


void Q_learning::print_Q()
{
    for(int i = 0; i < (states ); i++)
        {
            cout << Q[i] << " ";
        } // i
    cout << endl;
}


void Q_learning::update_states_visited(int s)
{
    states_visited[s] = 1;
    bool found = false;
    for (int i = 0; i < s_v_index.size(); i++)
        {
        if (states_visited == s_v_index[i])
            {
            found = true;
            }
        }

    if (!found)
        {
        s_v_index.push_back(states_visited);
        }
}

void Q_learning::update_reward_table(vector<int> s_v, int c_s)
{
    for (int i = 0; i < states; i++)/// travels through the states visited vector "colums"
        {
        if (s_v[i] == 1) /// if a state is visited, go into next for-loop
            {
            for (int j = 0; j < states; j++) /// rows
                {
                /// only allow to update positive values, because marbles are always positive
                if (reward_array[j][i] > 0)
                    {
                    reward_array[j][i] = 0; /// remove the marble
                    }
                }
            }
        }

}
void Q_learning::pick_Q_table(vector<int> s_v, int c_s)
{

    int index = find_index(s_v);
    Q=vec_3d_Q[index][c_s];

}

void Q_learning::save_Q_table(vector<int> s_v, int c_s)
{
    int index = find_index(s_v);
    vec_3d_Q[index][c_s]=Q;
}

void Q_learning::restart()
{
    vector<int> tmp(states,0);
    states_visited=tmp;
    current_state=0;
    update_states_visited(0);
    reward_array=reward_array0;
    runs_for_test=0;
    vector_of_chosen_actions.clear();
}



void Q_learning::clean_up()
{
    Q.clear();
    states_visited.clear();
    vec_3d_Q.clear();
    s_v_index.clear();

}


int Q_learning::find_index(vector<int> vec_to_find)
{
    int index=0;

    for(int i=0; i< s_v_index.size() /*pow(2,states)*/;i++)
        {
        if(s_v_index[i]==vec_to_find)
            {
            index=i;
            }
        }
    return index;
}

void Q_learning::print_runs_vector()
{
    save_data();
    cout << "the number of runs went like this: " << endl;

    for(int i=0;i<runs_vector.size();i++)
        {
        cout << runs_vector[i]<<", ";
        }

}

void Q_learning::update_epsilon()
{
    if (epsilon_var==iterations)
        {
        epsilon_times_ten=0;
        }

}

void Q_learning::print_latest_path()
{
    cout<< endl << "The LAST path looked like this: " << endl;

    for(int i=0;i<vector_of_chosen_actions_copy.size();i++)
        {
        cout << vector_of_chosen_actions_copy[i]<<", ";
        }
cout << endl;
}

void Q_learning::save_data()
{
    time_t varTime = time(0);
    string fileName = +"time_stamp" + to_string(varTime)+"runs:"+to_string(iterations)+"learning_rate:"+to_string(leaning_rate) + "gamma:"+to_string(gamma) +"_EPSILON:"+to_string(epsilon_times_ten)+".txt";
    ofstream outFile(fileName);
    for (const auto &e : runs_vector) outFile << e << "\n";
    for (const auto &e : vector_of_chosen_actions_copy) outFile << e << " ";
    outFile.close();
    cout << "Data saved as: " << fileName << endl;
}

void Q_learning::set_gamma(double ga)
{
    gamma=ga;
}

void Q_learning::set_leaning_rate(double lr)
{
    leaning_rate= lr;
}

int Q_learning::get_rand_small()
{
    static std::random_device randDev;
    static std::mt19937 twister(randDev());
    static std::uniform_int_distribution<int> dist;
    dist.param(std::uniform_int_distribution<int>::param_type(0, (states-1)));

    return dist(twister);
}

int Q_learning::get_rand_big()
{
    static std::random_device randDev;
    static std::mt19937 twister(randDev());
    static std::uniform_int_distribution<int> dist;
    dist.param(std::uniform_int_distribution<int>::param_type(0, 99));

    return dist(twister);
}

void Q_learning::set_epsilon(double en)
{
    epsilon_times_ten=en*100;
}


