//
// Created by lars on 11/17/19.
//

#include "../includes/Q_learning.h"
#define COUT 1
#define NEW_RANDOM 1
#define MASSIVE_DEBUG 0

#define METHOD_ONE 1
#define METHOD_TWO 0


Q_learning::Q_learning(int number_of_states, int i)
{
    cout << "Constructor: hello" << endl;
   ///
    vector<int> int_list_tmp;
    for (int i=1;i<=100;i++)
        {int_list_tmp.push_back(i);}


   ///
    states=number_of_states;

    actions=number_of_states;
    iterations=i;
    cout << "Constructor: alive 0" << endl;

    vector<int> Q_temp(states,0);
    Q=Q_temp;

    cout << "Constructor: alive 1" << endl;

#if NEW_RANDOM ==0
    srand(time(NULL));
#endif




    vector<int> tmp_goal(states,1);
    goal_states=tmp_goal;

    vector<int> tmp_visited(states,0);
    states_visited=tmp_visited;

   // current_state=initialState;

    reward_array=reward_array0;
    //vector<vector<int>> s_v_index_temp(1,vector<int>(states,0));
    //s_v_index=s_v_index_temp;
    cout << "Constructor: alive 2" << endl;
    update_states_visited(0);
    cout << "Constructor: alive 3" << endl;
///////////////////
//    vector<vector<int>> truth_table_temp(pow(2,states),vector<int>(states));
//    truth_table=truth_table_temp;

//    vector<int> truth_table_row(states);
//    generate_all_binary_strings(truth_table_row, 0)f;
//
    cout << "Constructor: alive 4" << endl;
    vector<vector<vector<int>>> vec_3d_temp(pow(2,states), vector<vector<int>>(states, vector<int>(states)));
    cout << "Constructor: alive 5" << endl;
//    vector<vector<vector<int>>> vec_3d_temp(1, vector<vector<int>>(states, vector<int>(states)));

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
//    return static_cast<int>(reward_array[current_state][action] + (gamma * maximum(action, false)));/////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void Q_learning::learning_trials()
{
    cout << "learning_trials(): hello " << endl;
    vector_of_chosen_actions.push_back(0);
    //vector<vector<int>> temp_vec_vec(states,vector<int>(states,0));
    //vec_3d_Q.push_back(temp_vec_vec);
    for(int j = 0; j <= (iterations - 1); j++)
        {
//        for(int i = 0; i <= (states - 1); i++)
            {
                episode(initialState);

                if(j==iterations-1)
                    {
                    vector_of_chosen_actions_copy=vector_of_chosen_actions;
                    }



#if COUT
//            if(runs_for_test<states_quad)
            cout << "The run through all states took " <<runs_for_test << " steps" <<endl;
           // print_all_Qs();
//           if(runs_for_test==states_quad)
//               cout << "Step limit of " << states_quad << " reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
           cout << "the last state was: " << current_state << endl;
            cout << "****************************************************************************************************************"<<endl << endl;

#endif
//           cout<< "Hello"<<endl;
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

    //Travel from state to state until goal state is reached.
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
    //When goal_states == states_visited, run through the set once more to
    //for convergence.
//    for(int i = 0; i <= (actions - 1); i++){ /// ***??????????????????????
//        chooseAnAction();
//
//        } // i

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

    /// #1 fisrt time at the starting point:
    /// states_visited=1000 and curent_state=0.   pick_Q_table(states_visited,current_state); should pick the Q0 table as Q.

    /// case 2: by luck, the explore, will pick possible action=1, but for now nothing has changed
    ///states_visited=1000 and curent_state=0.   pick_Q_table(states_visited,current_state); should pick the Q0 table as Q.
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
                 current_state); ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if MASSIVE_DEBUG
   cout<< "current Q is" << endl;
    print_Q();
#endif
    update_epsilon();

    if (choice > (99 - epsilon_times_ten)) /// if choice is larger than 99-epsilon_times_ten, take random action
        {

        /// case 2: by luck, the explore, will pick possible action=1
        //Randomly choose a possible action connected to the current state.
        possibleAction = getRandomAction();
#if MASSIVE_DEBUG
        cout << "random action was chosen to: " << possibleAction <<endl<<endl;
#endif

        } else /// if choice is smaller than 99-epsilon_times_ten take best action
        {

        //Chose best known action
        /// first time this returns 0 and nothing happens
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
                                                                Q[possibleAction]);///////////////-----------------------------------------------------------------------------------------------------------------------
        const int old_state = current_state;
        //vector<int> states_visited_old=states_visited;
        current_state = possibleAction;
//        update_states_visited(current_state);

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
//        update_states_visited(current_state);

        save_Q_table(states_visited, old_state);

        update_states_visited(current_state);
        }
//        else if(reward_array[current_state][possibleAction] = 0)
//            {
//            Q[possibleAction] = Q[possibleAction]-100;///////////////-----------------------------------------------------------------------------------------------------------------------
////        Q[possibleAction] = (reward(possibleAction));
//
//            const int old_state = current_state;
//            //vector<int> states_visited_old=states_visited;
//            current_state = possibleAction;
//            /// case 2: store Q in Q0, by states_visited=1000 and old_state=0
//            save_Q_table(states_visited, old_state);
//
//            update_states_visited(current_state);
//
//        }
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


    /// first time Q is stored back to Q0, since neither states_visited or current_state has changed
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


}

int Q_learning::maximum(int state, bool returnIndexOnly)
{


    int winner = 0;
    bool foundNewWinner;
    bool done = false;

    //do {
        foundNewWinner = false;
        for(int i = 0; i <= (states - 1); i++)
            {

            if( i != winner ) //(i < winner) || (i > winner))
                {     //Avoid self-comparison.

                if(Q[i] > Q[winner]) ///
                    {
                    winner = i;
                    foundNewWinner = true;
                    }
                }
            } // i
/*
        if(foundNewWinner == false)
            {
            done = true;
            }
*/
      //  } while(done = false);

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

void Q_learning::run_tests()
{
//
//        current_state = initialState;
//        newState = 0;
//        do {
//            newState = maximum(current_state, true);
//            cout << current_state << ", ";
//            current_state = newState;
//            } while(current_state < goal_state);
//        cout << goal_state<< endl;


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

//                vector<vector<int>> temp_vec_vec(states,vector<int>(states,0));
        // vec_3d_Q.push_back(temp_vec_vec);
//                vec_3d_Q[s_v_index.size()-1]=temp_vec_vec;
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


    //vector<vector<int>> Q_temp(states,vector<int>(actions));
    //Q=Q_temp;

}

void Q_learning::print_Q_percentage()
{

//    int max_value=0;
//    for(int i = 0; i <= (states - 1); i++)
//        {
//        for (int j = 0; j <= (actions - 1); j++)
//            {
//            if (Q[j] > max_value)
//                max_value = Q[j];
//            }
//        }
//
//
//
//    for(int i = 0; i <= (states - 1); i++)
//        {
//        for(int j = 0; j <= (actions - 1); j++)
//            {
//            cout << (float)(Q[j]/(float)max_value)*100;
//            if(j < states - 1)
//                {
//                cout << " ; ";
//                }
//            } // j
//        cout << "\n";
//        } // i
//    cout << "\n";

}

void Q_learning::clean_up()
{
    Q.clear();
    states_visited.clear();
    vec_3d_Q.clear();
    s_v_index.clear();


}

void Q_learning::load_Q_with(int load)
{
    for(int i = 0; i <= (states - 1); i++)
        {
        for (int j = 0; j <= (actions - 1); j++)
            {
          //  if(Q[i][j]<=0)
                {
                Q[j] = load;
                }
            }
        }

}

void Q_learning::double_print()
{
    print_Q();
    print_Q_percentage();

}

void Q_learning::print_all_Qs()
{


//for(int h =0; h<s_v_index.size();h++)
//    {
//    for (int i = 0; i <= (states - 1); i++)
//        {
//        for (int j = 0; j <= (actions - 1); j++)
//            {
//            cout << vec_3d_Q[h][i][j];
//            if (j < states - 1)
//                {
//                cout << " ; ";
//                }
//            } // j
//        cout << "\n";
//        } // i
//    cout << "\n";
//    } // h
//    cout<< "states_visited: {1000} , current_state: 0 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[0][0][i] << " "; /// sv=0 , cs=0
//        } // i
//        cout <<endl;
//
//    cout<< "states_visited: {1100} , current_state: 1 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[1][1][i] << " "; /// sv=1 , cs=1
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1110} , current_state: 2 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[2][2][i] << " ";
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1110} , current_state: 1 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[2][1][i] << " ";
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1110} , current_state: 0 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[2][0][i] << " ";
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1101} , current_state: 3 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[3][3][i] << " ";
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1101} , current_state: 1 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[3][1][i] << " ";
//        } // i
//    cout <<endl;
//
//    cout<< "states_visited: {1101} , current_state: 0 " << endl;
//    for(int i = 0; i < (states ); i++)
//        {
//        cout << vec_3d_Q[3][0][i] << " ";
//        } // i
//    cout << endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[0][0][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[0][1][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[0][2][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[0][3][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[1][0][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[1][1][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[1][2][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[1][3][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[2][0][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[2][1][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[2][2][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[2][3][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[3][0][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[3][1][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[3][2][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;

    for(int i = 0; i < (states ); i++)
        {
        cout << vec_3d_Q[3][3][i] << " "; /// sv=0 , cs=0
        } // i
    cout <<endl;
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
//    if(epsilon_var<iterations/4)
//        {
//        epsilon_times_ten=100;
//        }
//    else if (epsilon_var<(iterations/1.5))
//        {
//        epsilon_times_ten=50;
//        }
//    else if (epsilon_var<(iterations/1.2))
//        {
//        epsilon_times_ten=20;
//        }
//    else if (epsilon_var<iterations)
//        {
//        epsilon_times_ten=10;
//        }
//    else
//        epsilon_times_ten=00;

/////////////////
//    if(epsilon_var<iterations/50)
//        {
//        epsilon_times_ten=100;
//        }
//    else if(epsilon_var<iterations/2)
//        {
//        epsilon_times_ten=50;
//        }
//    else if (epsilon_var<(iterations/1.5))
//        {
//        epsilon_times_ten=35;
//        }
//    else if (epsilon_var<(iterations/1.2))
//        {
//        epsilon_times_ten=20;
//        }

    if (epsilon_var<iterations)
        {
        ;
        }
    else
        epsilon_times_ten=0;
///////////////////



//    else if(epsilon_var<iterations-5)
//        {
//        epsilon_times_ten = 15;
//        }
//    else
//        {
//        epsilon_times_ten = 0;
//        }



//    if(epsilon_var<(iterations/20))
//        {
//        epsilon_times_ten=60;
//        }
//    else if(epsilon_var<(iterations/10))
//        {
//        epsilon_times_ten=30;
//
//        }
//
//    else if(epsilon_var<(iterations/4))
//        {
//        epsilon_times_ten=25;
//
//        }
//
//    else if(epsilon_var<(iterations/2))
//        {
//        epsilon_times_ten=15;
//        }
//
//    else if(epsilon_var<(iterations/1.5))
//        {
//        epsilon_times_ten=10;
//        }
//    else
//        {
//        epsilon_times_ten = 0;
//        }


}
void Q_learning::print_path()
{
    cout << "the path looked like this: " << endl;

    for(int i=0;i<vector_of_chosen_actions.size();i++)
        {
        cout << vector_of_chosen_actions[i]<<", ";
        }
    cout << endl;

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


//int Q_learning::get_rand(const int& a, const int& b)
//{
//    cout << "random hello"<< endl;
//    static std::random_device randDev;
//    cout << "random hello2"<< endl;
//    static std::mt19937 twister(randDev());
//    cout << "random hello3"<< endl;
//    static std::uniform_int_distribution<int> dist;
//    cout << "random hello4"<< endl;
//    dist.param(std::uniform_int_distribution<int>::param_type(a, b));
//    cout << "random hello5 "<< dist(twister)<< endl;
//
//    return dist(twister);
//}

