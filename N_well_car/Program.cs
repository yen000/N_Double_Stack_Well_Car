using System;
using System.Collections.Generic;
using System.IO;
using ILOG.CPLEX;
using ILOG.Concert;
using System.Collections;


namespace N_well_car
{
    class Program
    {
        static void Main(string[] args)
        {

            Console.WriteLine("<Program start>");

            string[] overall_file_path = new string[] { "dataset21", "dataset22", "dataset23", "dataset24", "dataset25", };

            for (int a = 0; a < overall_file_path.Length; a++)
            {

                do
                {

                    Console.WriteLine("3-stage-method");

                    List<List<double>> w20l = new List<List<double>>();
                    List<List<double>> w20e = new List<List<double>>();
                    List<List<double>> w40 = new List<List<double>>();
                    List<List<double>> w20l_second = new List<List<double>>();
                    List<List<double>> w20e_second = new List<List<double>>();
                    List<List<double>> w40_second = new List<List<double>>();

                    string file_path = overall_file_path[a] + "\\w20l.csv";


                    if (file_path != "none")
                    {
                        StreamReader w20l_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w20l_file.ReadLine();

                        while ((data = w20l_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w20l_second.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                            w20l.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });

                        }
                    }

                    file_path = overall_file_path[a] + "\\w20e.csv";

                    if (file_path != "none")
                    {
                        StreamReader w20e_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w20e_file.ReadLine();

                        while ((data = w20e_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w20e_second.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                            w20e.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });

                        }
                    }

                    file_path = overall_file_path[a] + "\\w40.csv";

                    if (file_path != "none")
                    {
                        StreamReader w40_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w40_file.ReadLine();

                        while ((data = w40_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w40_second.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                            w40.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });

                        }
                    }

                    file_path = overall_file_path[a] + "\\car.csv";

                    double[] weight_limit, weight_tolerence_factor;
                    int car_amount;
                    List<List<double>> car_info = new List<List<double>>();
                    List<List<double>> car_info_second = new List<List<double>>();

                    if (file_path.Length > 10)
                    {

                        StreamReader car_file = new StreamReader(file_path);

                        string[] values = null;

                        string car_data = car_file.ReadLine();

                        while ((car_data = car_file.ReadLine()) != null)
                        {
                            values = car_data.Split(',');
                            car_info.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                            car_info_second.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                        }

                        car_amount = car_info.Count;

                        weight_limit = new double[car_amount];
                        weight_tolerence_factor = new double[car_amount];

                        for (int i = 0; i < car_amount; i++)
                        {
                            weight_limit[i] = car_info[i][0];
                            weight_tolerence_factor[i] = car_info[i][1];
                        }

                    }
                    else
                    {

                        Console.Write("\n+---Set parameters---+\nSet the car's amount:\n> ");

                        car_amount = int.Parse(Console.ReadLine());

                        weight_limit = new double[car_amount];
                        weight_tolerence_factor = new double[car_amount];

                        Console.Write("\nSet the car's weight limit:\n> ");
                        weight_limit = initial_array(weight_limit, double.Parse(Console.ReadLine()));

                        Console.Write("\nSet the car's tolerence factor of (upper_weight/lower_weight)\n> ");
                        weight_tolerence_factor = initial_array(weight_tolerence_factor, double.Parse(Console.ReadLine()));

                    }
                    int stack_amount = 2;

                    string result_file_path = overall_file_path[a] + "\\three_stage_result.csv";


                    List<double> hub_set = get_hub_sets(w20l, w20e, w40);


                    double[] x_weights = new double[w20l.Count];

                    for (int i = 0; i < w20l.Count; i++)
                    {
                        x_weights[i] = w20l[i][0];
                    }

                    double[] v_weights = new double[w20e.Count];

                    for (int i = 0; i < w20e.Count; i++)
                    {
                        v_weights[i] = w20e[i][0];
                    }

                    double[] y_weights = new double[w40.Count];

                    for (int i = 0; i < w40.Count; i++)
                    {
                        y_weights[i] = w40[i][0];
                    }

                    int hub_amount = hub_set.Count;

                    int car_amount_all = car_amount;
                    // model

                    #region first stage
                    // declare variables etc.

                    Console.WriteLine("\n> Start building the model");

                    Cplex new_model = new Cplex();


                    INumVar[][] x_variables = new INumVar[x_weights.Length][];
                    INumVar[][] v_variables = new INumVar[v_weights.Length][];
                    INumVar[][][] y_variables = new INumVar[y_weights.Length][][];
                    INumVar[][] u_variables = new INumVar[hub_set.Count][];
                    INumVar[] z_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    INumVar[] t_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                    for (int i = 0; i < x_weights.Length; i++)
                    {
                        x_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    for (int i = 0; i < v_weights.Length; i++)
                    {
                        v_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    for (int i = 0; i < y_weights.Length; i++)
                    {
                        y_variables[i] = new INumVar[car_amount][];

                        for (int j = 0; j < car_amount; j++)
                        {

                            y_variables[i][j] = new_model.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                        }
                    }

                    for (int i = 0; i < hub_set.Count; i++)
                    {
                        u_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    // declare the objective function

                    Console.WriteLine("[ 0/14]::Building objective function");




                    // declare the objective function


                    ILinearNumExpr y_total_cost = new_model.LinearNumExpr();


                    for (int i = 0; i < y_weights.Length; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {

                            y_total_cost.AddTerm(1, y_variables[i][j][0]);

                        }
                    }

                    new_model.AddMaximize(y_total_cost);

                    // declare constraints

                    int x_amount = x_weights.Length, v_amount = v_weights.Length, y_amount = y_weights.Length;

                    // constraint 2a

                    Console.WriteLine("[ 1/14]::Building constraint1a");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2a.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2a.AddTerm(-2, z_variables[i]);

                            new_model.AddLe(constraint2a, 0);

                            constraint2a.Clear();

                        }
                    }
                    // constraint 2b

                    Console.WriteLine("[ 1/14]::Building constraint1b");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2b.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2b.AddTerm(-2, u_variables[h][i]);

                            new_model.AddLe(constraint2b, 0);

                            constraint2b.Clear();

                        }
                    }

                    // constraint 2c

                    Console.WriteLine("[ 1/14]::Building constraint1c");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2c.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2c.AddTerm(-2, u_variables[h][i]);
                            constraint2c.AddTerm(-2, z_variables[i]);

                            new_model.AddGe(constraint2c, -2);

                            constraint2c.Clear();

                        }
                    }

                    // constraint 3

                    Console.WriteLine("[ 2/14]::Building constraint2");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < x_amount; i++)
                        {
                            if (w20l[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    constraint3.AddTerm(1, x_variables[i][j]);
                                }

                                new_model.AddLe(constraint3, 1);

                                constraint3.Clear();
                            }
                        }
                    }

                    // constraint 4a 

                    Console.WriteLine("[ 3/14]::Building constraint3a");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4a.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4a.AddTerm(-2, t_variables[i]);

                            new_model.AddLe(constraint4a, 0);

                            constraint4a.Clear();

                        }
                    }

                    // constraint 4b

                    Console.WriteLine("[ 3/14]::Building constraint3b");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4b.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4b.AddTerm(-2, u_variables[h][i]);

                            new_model.AddLe(constraint4b, 0);

                            constraint4b.Clear();

                        }
                    }

                    // constraint 4c

                    Console.WriteLine("[ 3/14]::Building constraint3c");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4c.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4c.AddTerm(-2, u_variables[h][i]);
                            constraint4c.AddTerm(-2, t_variables[i]);

                            new_model.AddGe(constraint4c, -2);

                            constraint4c.Clear();

                        }
                    }

                    // constraint 5 

                    Console.WriteLine("[ 4/14]::Building constraint4");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < v_amount; i++)
                        {
                            if (w20e[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    constraint5.AddTerm(1, v_variables[i][j]);
                                }


                                new_model.AddLe(constraint5, 1);

                                constraint5.Clear();
                            }
                        }
                    }
                    // constraint6 

                    Console.WriteLine("[ 5/14]::Building constraint5");

                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                        constraint6.AddTerm(1, z_variables[i]);
                        constraint6.AddTerm(1, t_variables[i]);

                        new_model.AddLe(constraint6, 1);

                        constraint6.Clear();
                    }


                    // constraint 7

                    Console.WriteLine("[ 6/14]::Building constraint6");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < y_amount; i++)
                        {
                            if (w40[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    for (int k = 0; k < stack_amount; k++)
                                    {
                                        constraint7.AddTerm(1, y_variables[i][j][k]);
                                    }
                                }

                                new_model.AddLe(constraint7, 1);

                                constraint7.Clear();
                            }
                        }
                    }

                    // constraint 8

                    Console.WriteLine("[ 7/14]::Building constraint7");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint8.AddTerm(1, y_variables[j][i][0]);
                                }
                            }

                            constraint8.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint8, 0);
                        }
                    }
                    // constraint 9a

                    Console.WriteLine("[ 8/14]::Building constraint8a");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint9a.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint9a.AddTerm(1, z_variables[i]);

                            new_model.AddLe(constraint9a, 1);
                        }
                    }

                    // constraint 9b

                    Console.WriteLine("[ 8/14]::Building constraint8b");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {

                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint9b.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint9b.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint9b, 0);
                        }
                    }

                    // constraint 10a

                    Console.WriteLine("[ 9/14]::Building constraint9");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint10a.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint10a.AddTerm(1, t_variables[i]);

                            new_model.AddLe(constraint10a, 1);
                        }
                    }

                    // constraint 10b

                    Console.WriteLine("[10/14]::Building constraint10");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint10b.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint10b.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint10b, 0);
                        }
                    }


                    // constraint 11

                    Console.WriteLine("[11/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    for (int k = 0; k < stack_amount; k++)
                                    {
                                        constraint11.AddTerm(y_weights[j], y_variables[j][i][k]);
                                    }
                                }
                            }

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                                }
                            }

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                                }
                            }

                            new_model.AddLe(constraint11, weight_limit[i]);

                            constraint11.Clear();
                        }
                    }

                    // constraint 12

                    Console.WriteLine("[12/14]::Building constraint12");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint12 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(-1 * y_weights[j], y_variables[j][i][0]);
                                    constraint12.AddTerm(weight_tolerence_factor[i] * y_weights[j], y_variables[j][i][1]);
                                }
                            }

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(weight_tolerence_factor[i] * x_weights[j], x_variables[j][i]);
                                }
                            }

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(weight_tolerence_factor[i] * v_weights[j], v_variables[j][i]);
                                }
                            }

                            new_model.AddGe(constraint12, 0);

                            constraint12.Clear();
                        }
                    }


                    //constraint 13

                    Console.WriteLine("[13/14]::Building constraint13");

                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                        for (int h = 0; h < hub_amount; h++)
                        {
                            constraint13.AddTerm(1, u_variables[h][i]);
                        }

                        new_model.AddEq(constraint13, 1);

                        constraint13.Clear();
                    }

                    //constraint 14 NEW CONSTRAINT 19

                    Console.WriteLine("[14/14]::Building constraint14");

                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint14 = new_model.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {

                            constraint14.AddTerm(0.5, x_variables[j][i]);
                        }

                        for (int j = 0; j < v_amount; j++)
                        {

                            constraint14.AddTerm(0.5, v_variables[j][i]);

                        }

                        for (int j = 0; j < y_amount; j++)
                        {

                            constraint14.AddTerm(-1, y_variables[j][i][0]);
                            constraint14.AddTerm(1, y_variables[j][i][1]);

                        }
                        new_model.AddEq(constraint14, 0);

                        constraint14.Clear();

                    }


                    // model-solving

                    Console.WriteLine("\n> Start solving the model");

                    System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

                    sw.Reset(); sw.Start();


                    new_model.Solve();
                    sw.Stop();

                    double time1 = sw.Elapsed.TotalMilliseconds;

                    // get the result variable values

                    double[][] x_result = new double[x_amount][];

                    for (int i = 0; i < x_amount; i++)
                    {
                        x_result[i] = new_model.GetValues(x_variables[i]);
                    }

                    double[][] v_result = new double[v_amount][];

                    for (int i = 0; i < v_amount; i++)
                    {
                        v_result[i] = new_model.GetValues(v_variables[i]);
                    }

                    double[][][] y_result = new double[y_amount][][];

                    for (int i = 0; i < y_amount; i++)
                    {
                        y_result[i] = new double[car_amount][];

                        for (int j = 0; j < car_amount; j++)
                        {
                            y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                        }
                    }

                    double[][] u_result = new double[hub_amount][];

                    for (int i = 0; i < hub_amount; i++)
                    {
                        u_result[i] = new_model.GetValues(u_variables[i]);
                    }

                    // transform result into readable form


                    double[][][] loading = new double[car_amount][][];

                    for (int i = 0; i < car_amount; i++)
                    {
                        loading[i] = new double[6][];

                        for (int j = 0; j < 6; j++)
                        {
                            loading[i][j] = new double[2];
                        }
                    }

                    List<List<double>> container_destination_list_x = new List<List<double>>();
                    List<List<double>> container_destination_list_v = new List<List<double>>();
                    List<List<double>> container_destination_list_y = new List<List<double>>();
                    List<double> car_info_list = new List<double>();
                    for (int i = 0; i < car_info.Count; i++)
                    {
                        car_info_list.Add(i);
                    }


                    // record x-result
                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < x_amount; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {
                                if (x_result[i][j] == 1 && u_result[h][j] == 1 && find_destination(u_result, j) != -1)
                                {
                                    if (loading[j][0][0] == 0)
                                    {
                                        loading[j][0][0] = i + 1;
                                        loading[j][0][1] = x_weights[i];

                                        container_destination_list_x.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] });

                                    }


                                    else
                                    {
                                        loading[j][1][0] = i + 1;
                                        loading[j][1][1] = x_weights[i];
                                        container_destination_list_x.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] }); ;
                                    }
                                }
                            }
                        }
                    }



                    // record v-result
                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < v_amount; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {
                                if (v_result[i][j] == 1 && u_result[h][j] == 1 && find_destination(u_result, j) != -1)
                                {
                                    if (loading[j][2][0] == 0)
                                    {
                                        loading[j][2][0] = i + 1;
                                        loading[j][2][1] = v_weights[i];
                                        container_destination_list_v.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] });
                                    }
                                    else
                                    {
                                        loading[j][3][0] = i + 1;
                                        loading[j][3][1] = v_weights[i];
                                        container_destination_list_v.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] });
                                    }
                                }
                            }
                        }
                    }

                    // record y-result
                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < y_amount; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {
                                if (y_result[i][j][1] == 1 && u_result[h][j] == 1 && find_destination(u_result, j) != -1)
                                {
                                    if (loading[j][4][0] == 0)
                                    {
                                        loading[j][4][0] = i + 1;
                                        loading[j][4][1] = y_weights[i];
                                        container_destination_list_y.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] });

                                    }
                                    else
                                    {
                                        Console.WriteLine("there's an error! #40's upper");
                                        Console.Read();
                                    }
                                }
                                if (y_result[i][j][0] == 1 && u_result[h][j] == 1)
                                {
                                    if (loading[j][5][0] == 0)
                                    {
                                        loading[j][5][0] = i + 1;
                                        loading[j][5][1] = y_weights[i];
                                        container_destination_list_y.Add(new List<double> { i + 1, hub_set[find_destination(u_result, j)] });
                                    }
                                    else
                                    {
                                        Console.WriteLine("there's an error! #40's lower");
                                        Console.Read();
                                    }
                                }
                            }
                        }
                    }

                    List<double> w20l_container_list = new List<double>();
                    List<double> w20e_container_list = new List<double>();
                    List<double> w40_container_list = new List<double>();

                    w20l_container_list = container_amount(w20l);
                    w20e_container_list = container_amount(w20e);
                    w40_container_list = container_amount(w40);

                    // output the result

                    StreamWriter csv_output = new StreamWriter(result_file_path);
                    csv_output.WriteLine("First stage");
                    csv_output.WriteLine("Car,20L,20L,20E,20E,40lower,40upper,,weight utility,space utility");

                    Console.WriteLine("\n+---result---+");
                    List<double> one = new List<double>();


                    for (int i = 0; i < car_amount; i++)
                    {
                        string output = "", temp = "", rank = "";
                        double total_weight = 0;
                        double space_utility = 0;

                        for (int j = 0; j < 6; j++)
                        {
                            if (loading[i][j][0] != 0)
                            {

                                switch (j)
                                {
                                    case 0:
                                    case 1:
                                        temp = "x";
                                        space_utility++;
                                        w20l_container_list.Remove(loading[i][j][0]);

                                        for (int k = 0; k <= w20l.Count - 1; k++)
                                        {
                                            for (int x = 0; x < container_destination_list_x.Count; x++)
                                            {
                                                if (loading[i][j][0] == container_destination_list_x[x][0])
                                                {
                                                    if (loading[i][j][1] == w20l[k][0] && container_destination_list_x[x][1] == w20l[k][1])
                                                    {
                                                        w20l.RemoveAt(k);
                                                        k = w20l.Count;
                                                        break;
                                                    }
                                                }
                                            }
                                        }

                                        break;
                                    case 2:
                                    case 3:
                                        temp = "v";
                                        space_utility++;
                                        w20e_container_list.Remove(loading[i][j][0]);

                                        for (int k = 0; k <= w20e.Count - 1; k++)
                                        {
                                            for (int x = 0; x < container_destination_list_v.Count; x++)
                                            {
                                                if (loading[i][j][0] == container_destination_list_v[x][0])
                                                {
                                                    if (loading[i][j][1] == w20e[k][0] && container_destination_list_v[x][1] == w20e[k][1])
                                                    {
                                                        w20e.RemoveAt(k);
                                                        k = w20e.Count;
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                        //}
                                        break;
                                    case 4:
                                    case 5:
                                        temp = "y";
                                        space_utility += 2;
                                        w40_container_list.Remove(loading[i][j][0]);

                                        for (int k = 0; k <= w40.Count - 1; k++)
                                        {
                                            for (int x = 0; x < container_destination_list_y.Count; x++)
                                            {
                                                if (loading[i][j][0] == container_destination_list_y[x][0])
                                                {
                                                    if (loading[i][j][1] == w40[k][0] && container_destination_list_y[x][1] == w40[k][1])
                                                    {
                                                        w40.RemoveAt(k);
                                                        k = w40.Count;
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                        break;
                                }

                                output += temp + loading[i][j][0].ToString();
                                total_weight += loading[i][j][1];

                            }
                            else
                            {
                                output += "-";
                            }
                            if (j < 5)
                            {
                                output += ", ";
                            }
                        }

                        switch (i)
                        {
                            case 0:
                                rank = "st";
                                if (total_weight > 0)
                                {
                                    for (int z = 0; z < car_info.Count; z++)
                                    {
                                        if (car_info_list[z] == i)
                                        {
                                            car_info.RemoveAt(z);
                                            car_info_list.RemoveAt(z);
                                            break;

                                        }
                                    }

                                }
                                break;

                            case 1:
                                rank = "nd";
                                if (total_weight > 0)
                                {
                                    for (int z = 0; z < car_info.Count; z++)
                                    {
                                        if (car_info_list[z] == i)
                                        {
                                            car_info.RemoveAt(z);
                                            car_info_list.RemoveAt(z);
                                            break;

                                        }
                                    }
                                }

                                break;

                            case 2:
                                rank = "rd";
                                if (total_weight > 0)
                                {
                                    for (int z = 0; z < car_info.Count; z++)
                                    {
                                        if (car_info_list[z] == i)
                                        {
                                            car_info.RemoveAt(z);
                                            car_info_list.RemoveAt(z);
                                            break;

                                        }
                                    }
                                }

                                break;

                            default:
                                rank = "th";
                                if (total_weight > 0)
                                {
                                    for (int z = 0; z < car_info.Count; z++)
                                    {
                                        if (car_info_list[z] == i)
                                        {
                                            car_info.RemoveAt(z);
                                            car_info_list.RemoveAt(z);
                                            break;

                                        }
                                    }
                                }

                                break;
                        }

                        string destination;

                        if (find_destination(u_result, i) != -1)
                        {
                            destination = hub_set[find_destination(u_result, i)].ToString();
                        }
                        else
                        {
                            destination = "none";
                        }

                        Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                            weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4)");

                        csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + total_weight.ToString() + "/" +
                            weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                    }


                    double u1 = new_model.GetObjValue() / car_amount;
                    Console.WriteLine("+------------+\n");
                    Console.WriteLine("Finish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                        " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####") + "\n" + "utility: " + u1.ToString() + "\n");

                    double ob1 = new_model.GetObjValue();

                    csv_output.WriteLine("\ncontainers that are not on rail cars:");



                    csv_output.WriteLine("(1) 20l:" + w20l_container_list.Count + " containers left");
                    for (int i = 0; i <= w20l_container_list.Count - 1; i++)
                    {
                        csv_output.Write("x" + w20l_container_list[i] + ",");
                    }

                    csv_output.WriteLine();

                    csv_output.WriteLine("(2) 20e:" + w20e_container_list.Count + " containers left");
                    for (int i = 0; i <= w20e_container_list.Count - 1; i++)
                    {
                        csv_output.Write("v" + w20e_container_list[i] + ",");
                    }

                    csv_output.WriteLine();

                    csv_output.WriteLine("(3) 40:" + w40_container_list.Count + " containers left");
                    for (int i = 0; i <= w40_container_list.Count - 1; i++)
                    {
                        csv_output.Write("y" + w40_container_list[i] + ",");
                    }

                    csv_output.Write("\ntime used(sec)," + (sw.Elapsed.TotalMilliseconds / 1000) + ",,obj-value," + new_model.GetObjValue() + ",,utility:," + u1 + "\n");

                    csv_output.WriteLine("-\nSecond stage");

                    int car_amount1 = (int)new_model.GetObjValue();
                    car_amount = car_amount - (int)new_model.GetObjValue();

                    new_model.End();

                    #endregion
                    #region second stage

                    double time2 = 0;
                    if (car_amount > 0)
                    {
                        stack_amount = 1;
                        #region constraint
                        x_weights = new double[w20l.Count];

                        for (int i = 0; i < w20l.Count; i++)
                        {
                            x_weights[i] = w20l[i][0];
                        }

                        v_weights = new double[w20e.Count];

                        for (int i = 0; i < w20e.Count; i++)
                        {
                            v_weights[i] = w20e[i][0];
                        }

                        y_weights = new double[w40.Count];

                        for (int i = 0; i < w40.Count; i++)
                        {
                            y_weights[i] = w40[i][0];
                        }
                        hub_set = get_hub_sets(w20l, w20e, w40);

                        hub_amount = hub_set.Count;

                        x_amount = x_weights.Length;
                        v_amount = v_weights.Length;
                        y_amount = y_weights.Length;

                        double[] weight_limit_ = new double[car_amount];
                        weight_tolerence_factor = new double[car_amount];

                        for (int i = 0; i < car_amount; i++)
                        {
                            weight_limit_[i] = car_info[i][0];
                            weight_tolerence_factor[i] = car_info[i][1];
                        }
                        //////////
                        new_model = new Cplex();

                        x_variables = new INumVar[x_weights.Length][];
                        v_variables = new INumVar[v_weights.Length][];
                        y_variables = new INumVar[y_weights.Length][][];
                        u_variables = new INumVar[hub_set.Count][];
                        z_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                        t_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                        for (int i = 0; i < x_weights.Length; i++)
                        {
                            x_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                        }

                        for (int i = 0; i < v_weights.Length; i++)
                        {
                            v_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);

                        }

                        for (int i = 0; i < y_weights.Length; i++)
                        {

                            y_variables[i] = new INumVar[car_amount][];

                            for (int j = 0; j < car_amount; j++)
                            {
                                y_variables[i][j] = new_model.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                            }

                        }

                        for (int i = 0; i < hub_set.Count; i++)
                        {
                            u_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                        }

                        // declare the objective function

                        Console.WriteLine("[ 0/14]::Building objective function");

                        ILinearNumExpr x_total_cost = new_model.LinearNumExpr();
                        ILinearNumExpr v_total_cost = new_model.LinearNumExpr();
                        y_total_cost = new_model.LinearNumExpr();

                        double multiplier = 1 / (4 * (double)car_amount);

                        for (int i = 0; i < x_weights.Length; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {
                                x_total_cost.AddTerm(multiplier, x_variables[i][j]);
                            }
                        }

                        for (int i = 0; i < v_weights.Length; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {
                                v_total_cost.AddTerm(multiplier, v_variables[i][j]);
                            }
                        }

                        for (int i = 0; i < y_weights.Length; i++)
                        {
                            for (int j = 0; j < car_amount; j++)
                            {

                                y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][0]);

                            }
                        }

                        new_model.AddMaximize(new_model.Sum(new_model.Sum(x_total_cost, v_total_cost), y_total_cost));


                        // constraint 2a 

                        Console.WriteLine("[ 1/12]::Building constraint1a");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                                for (int j = 0; j < x_amount; j++)
                                {
                                    if (w20l[j][1] == hub_set[h])
                                    {
                                        constraint2a.AddTerm(1, x_variables[j][i]);
                                    }
                                }

                                constraint2a.AddTerm(-2, z_variables[i]);

                                new_model.AddLe(constraint2a, 0);

                                constraint2a.Clear();

                            }
                        }
                        // constraint 2b

                        Console.WriteLine("[ 1/12]::Building constraint1b");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                                for (int j = 0; j < x_amount; j++)
                                {
                                    if (w20l[j][1] == hub_set[h])
                                    {
                                        constraint2b.AddTerm(1, x_variables[j][i]);
                                    }
                                }

                                constraint2b.AddTerm(-2, u_variables[h][i]);

                                new_model.AddLe(constraint2b, 0);

                                constraint2b.Clear();

                            }
                        }

                        // constraint 2c 

                        Console.WriteLine("[ 1/12]::Building constraint1c");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                                for (int j = 0; j < x_amount; j++)
                                {
                                    if (w20l[j][1] == hub_set[h])
                                    {
                                        constraint2c.AddTerm(1, x_variables[j][i]);
                                    }
                                }

                                constraint2c.AddTerm(-2, u_variables[h][i]);
                                constraint2c.AddTerm(-2, z_variables[i]);

                                new_model.AddGe(constraint2c, -2);

                                constraint2c.Clear();

                            }
                        }

                        // constraint 3 

                        Console.WriteLine("[ 2/12]::Building constraint2");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < x_amount; i++)
                            {
                                if (w20l[i][1] == hub_set[h])
                                {
                                    ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                                    for (int j = 0; j < car_amount; j++)
                                    {
                                        constraint3.AddTerm(1, x_variables[i][j]);
                                    }

                                    new_model.AddLe(constraint3, 1);

                                    constraint3.Clear();
                                }
                            }
                        }

                        // constraint 4a 

                        Console.WriteLine("[ 3/12]::Building constraint3a");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                                for (int j = 0; j < v_amount; j++)
                                {
                                    if (w20e[j][1] == hub_set[h])
                                    {
                                        constraint4a.AddTerm(1, v_variables[j][i]);
                                    }
                                }

                                constraint4a.AddTerm(-2, t_variables[i]);

                                new_model.AddLe(constraint4a, 0);

                                constraint4a.Clear();

                            }
                        }

                        // constraint 4b

                        Console.WriteLine("[ 3/12]::Building constraint3b");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                                for (int j = 0; j < v_amount; j++)
                                {
                                    if (w20e[j][1] == hub_set[h])
                                    {
                                        constraint4b.AddTerm(1, v_variables[j][i]);
                                    }
                                }

                                constraint4b.AddTerm(-2, u_variables[h][i]);

                                new_model.AddLe(constraint4b, 0);

                                constraint4b.Clear();

                            }
                        }

                        // constraint 4c

                        Console.WriteLine("[ 3/12]::Building constraint3c");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                                for (int j = 0; j < v_amount; j++)
                                {
                                    if (w20e[j][1] == hub_set[h])
                                    {
                                        constraint4c.AddTerm(1, v_variables[j][i]);
                                    }
                                }

                                constraint4c.AddTerm(-2, u_variables[h][i]);
                                constraint4c.AddTerm(-2, t_variables[i]);

                                new_model.AddGe(constraint4c, -2);

                                constraint4c.Clear();

                            }
                        }

                        // constraint 5 

                        Console.WriteLine("[ 4/12]::Building constraint4");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < v_amount; i++)
                            {
                                if (w20e[i][1] == hub_set[h])
                                {
                                    ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                                    for (int j = 0; j < car_amount; j++)
                                    {
                                        constraint5.AddTerm(1, v_variables[i][j]);
                                    }


                                    new_model.AddLe(constraint5, 1);

                                    constraint5.Clear();
                                }
                            }
                        }
                        // constraint 6 

                        Console.WriteLine("[ 5/12]::Building constraint5");

                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                            constraint6.AddTerm(1, z_variables[i]);
                            constraint6.AddTerm(1, t_variables[i]);

                            new_model.AddLe(constraint6, 1);

                            constraint6.Clear();
                        }


                        // constraint 7

                        Console.WriteLine("[ 6/12]::Building constraint6");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < y_amount; i++)
                            {
                                if (w40[i][1] == hub_set[h])
                                {
                                    ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                                    for (int j = 0; j < car_amount; j++)
                                    {
                                        constraint7.AddTerm(1, y_variables[i][j][0]);

                                    }

                                    new_model.AddLe(constraint7, 1);

                                    constraint7.Clear();
                                }
                            }
                        }

                        // constraint 8

                        /*Console.WriteLine("[ 9/14]::Building constraint8");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {
                                    if (w40[j][1] == hub_set[h])
                                    {
                                        constraint8.AddTerm(1, y_variables[j][i][0]);
                                    }
                                }

                                constraint8.AddTerm(-1, u_variables[h][i]);

                                new_model.AddLe(constraint8, 0);
                            }
                        }*/
                        // constraint 9a 

                        Console.WriteLine("[ 7/12]::Building constraint7");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {
                                    if (w40[j][1] == hub_set[h])
                                    {
                                        constraint9a.AddTerm(1, y_variables[j][i][0]);
                                    }
                                }

                                constraint9a.AddTerm(1, z_variables[i]);

                                new_model.AddLe(constraint9a, 1);
                            }
                        }

                        // constraint 9b

                        Console.WriteLine("[ 8/12]::Building constraint8");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {

                                    if (w40[j][1] == hub_set[h])
                                    {
                                        constraint9b.AddTerm(1, y_variables[j][i][0]);
                                    }
                                }

                                constraint9b.AddTerm(-1, u_variables[h][i]);

                                new_model.AddLe(constraint9b, 0);
                            }
                        }

                        // constraint 10a

                        Console.WriteLine("[ 9/12]::Building constraint9");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {
                                    if (w40[j][1] == hub_set[h])
                                    {
                                        constraint10a.AddTerm(1, y_variables[j][i][0]);
                                    }
                                }

                                constraint10a.AddTerm(1, t_variables[i]);

                                new_model.AddLe(constraint10a, 1);
                            }
                        }

                        // constraint 10b

                        Console.WriteLine("[10/12]::Building constraint10");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {
                                    if (w40[j][1] == hub_set[h])
                                    {
                                        constraint10b.AddTerm(1, y_variables[j][i][0]);
                                    }
                                }

                                constraint10b.AddTerm(-1, u_variables[h][i]);

                                new_model.AddLe(constraint10b, 0);
                            }
                        }


                        // constraint 11

                        Console.WriteLine("[11/12]::Building constraint11");

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                                for (int j = 0; j < y_amount; j++)
                                {
                                    if (w40[j][1] == hub_set[h])
                                    {

                                        constraint11.AddTerm(y_weights[j], y_variables[j][i][0]);

                                    }
                                }

                                for (int j = 0; j < x_amount; j++)
                                {
                                    if (w20l[j][1] == hub_set[h])
                                    {
                                        constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                                    }
                                }

                                for (int j = 0; j < v_amount; j++)
                                {
                                    if (w20e[j][1] == hub_set[h])
                                    {
                                        constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                                    }
                                }

                                new_model.AddLe(constraint11, weight_limit_[i]);

                                constraint11.Clear();
                            }
                        }

                        //constraint 13

                        Console.WriteLine("[12/12]::Building constraint12");

                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                            for (int h = 0; h < hub_amount; h++)
                            {
                                constraint13.AddTerm(1, u_variables[h][i]);
                            }

                            new_model.AddEq(constraint13, 1);

                            constraint13.Clear();
                        }

                        Console.WriteLine("\n> Start solving the model again!!!");

                        sw = new System.Diagnostics.Stopwatch();

                        sw.Reset(); sw.Start();

                        new_model.Solve();

                        sw.Stop();

                        // get the result variable values

                        time2 = sw.Elapsed.TotalMilliseconds;
                        double ob2 = new_model.GetObjValue();

                        Console.WriteLine("\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                            " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));

                        // get the result variable values

                        x_result = new double[x_amount][];

                        for (int i = 0; i < x_amount; i++)
                        {
                            x_result[i] = new_model.GetValues(x_variables[i]);
                        }

                        v_result = new double[v_amount][];

                        for (int i = 0; i < v_amount; i++)
                        {
                            v_result[i] = new_model.GetValues(v_variables[i]);
                        }

                        y_result = new double[y_amount][][];

                        for (int i = 0; i < y_amount; i++)
                        {
                            y_result[i] = new double[car_amount][];

                            for (int j = 0; j < car_amount; j++)
                            {
                                y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                            }
                        }

                        double[][] u_result_second = new double[hub_amount][];

                        for (int i = 0; i < hub_amount; i++)
                        {
                            u_result_second[i] = new_model.GetValues(u_variables[i]);
                        }


                        // record x-result
                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < x_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (x_result[i][j] == 1 && u_result_second[h][j] == 1 && find_destination(u_result_second, j) != -1)
                                    {
                                        if (loading[(int)car_info_list[j]][0][0] == 0)
                                        {
                                            loading[(int)car_info_list[j]][0][0] = w20l_container_list[i];
                                            loading[(int)car_info_list[j]][0][1] = x_weights[i];

                                            container_destination_list_x.Add(new List<double> { i + 1, hub_set[find_destination(u_result_second, j)] });

                                        }


                                        else
                                        {
                                            loading[(int)car_info_list[j]][1][0] = w20l_container_list[i];
                                            loading[(int)car_info_list[j]][1][1] = x_weights[i];
                                            container_destination_list_x.Add(new List<double> { i + 1, hub_set[find_destination(u_result_second, j)] }); ;
                                        }
                                    }
                                }
                            }
                        }



                        // record v-result
                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < v_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (v_result[i][j] == 1 && u_result_second[h][j] == 1 && find_destination(u_result_second, j) != -1)
                                    {
                                        if (loading[(int)car_info_list[j]][2][0] == 0)
                                        {
                                            loading[(int)car_info_list[j]][2][0] = w20e_container_list[i];
                                            loading[(int)car_info_list[j]][2][1] = v_weights[i];
                                            container_destination_list_v.Add(new List<double> { i + 1, hub_set[find_destination(u_result_second, j)] });
                                        }
                                        else
                                        {
                                            loading[(int)car_info_list[j]][3][0] = w20e_container_list[i];
                                            loading[(int)car_info_list[j]][3][1] = v_weights[i];
                                            container_destination_list_v.Add(new List<double> { i + 1, hub_set[find_destination(u_result_second, j)] });
                                        }
                                    }
                                }
                            }
                        }

                        // record y-result

                        for (int h = 0; h < hub_amount; h++)
                        {
                            for (int i = 0; i < y_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (y_result[i][j][0] == 1 && u_result_second[h][j] == 1 && find_destination(u_result_second, j) != -1)
                                    {
                                        if (loading[(int)car_info_list[j]][5][0] == 0)
                                        {
                                            loading[(int)car_info_list[j]][4][0] = w40_container_list[i];
                                            loading[(int)car_info_list[j]][4][1] = y_weights[i];
                                            container_destination_list_y.Add(new List<double> { i + 1, hub_set[find_destination(u_result_second, j)] });

                                        }
                                        else
                                        {
                                            Console.WriteLine("there's an error! #40's upper");
                                            Console.Read();
                                        }
                                    }
                                }
                            }
                        }

                        #endregion
                        Console.WriteLine("\n+---result---+");

                        List<double> w20l_container_list_overall = w20l_container_list; //overall means remains after second stage
                        List<double> w20e_container_list_overall = w20e_container_list;
                        List<double> w40_container_list_overall = w40_container_list;

                        w20l_container_list = container_amount(w20l); //重製container者
                        w20e_container_list = container_amount(w20e);
                        w40_container_list = container_amount(w40);
                        csv_output.WriteLine("Car,20L,20L,20E,20E,40lower,40upper,,weight utility,space utility");

                        for (int i = 0; i < loading.Length; i++)
                        {
                            string output = "", temp = "", rank = "";
                            double total_weight = 0;
                            double space_utility = 0;

                            for (int j = 0; j < 6; j++)
                            {
                                if (loading[i][j][0] != 0)
                                {

                                    switch (j)
                                    {
                                        case 0:
                                        case 1:
                                            temp = "x";
                                            space_utility++;
                                            w20l_container_list_overall.Remove(loading[i][j][0]);


                                            for (int k = 0; k <= w20l.Count - 1; k++)
                                            {
                                                for (int x = 0; x < container_destination_list_x.Count; x++)
                                                {
                                                    if (loading[i][j][0] == container_destination_list_x[x][0])
                                                    {
                                                        if (loading[i][j][1] == w20l[k][0] && container_destination_list_x[x][1] == w20l[k][1])
                                                        {
                                                            w20l.RemoveAt(k);
                                                            k = w20l.Count;
                                                            break;
                                                        }
                                                    }
                                                }
                                            }

                                            break;
                                        case 2:
                                        case 3:
                                            temp = "v";
                                            space_utility++;

                                            w20e_container_list_overall.Remove(loading[i][j][0]);


                                            for (int k = 0; k <= w20e.Count - 1; k++)
                                            {
                                                for (int x = 0; x < container_destination_list_v.Count; x++)
                                                {
                                                    if (loading[i][j][0] == container_destination_list_v[x][0])
                                                    {
                                                        if (loading[i][j][1] == w20e[k][0] && container_destination_list_v[x][1] == w20e[k][1])
                                                        {
                                                            w20e.RemoveAt(k);
                                                            k = w20e.Count;
                                                            break;
                                                        }
                                                    }
                                                }
                                            }

                                            break;
                                        case 4:
                                        case 5:
                                            temp = "y";
                                            space_utility += 2;

                                            w40_container_list_overall.Remove(loading[i][j][0]);

                                            for (int k = 0; k <= w40.Count - 1; k++)
                                            {
                                                for (int x = 0; x < container_destination_list_y.Count; x++)
                                                {
                                                    if (loading[i][j][0] == container_destination_list_y[x][0])
                                                    {
                                                        if (loading[i][j][1] == w40[k][0] && container_destination_list_y[x][1] == w40[k][1])
                                                        {
                                                            w40.RemoveAt(k);
                                                            k = w40.Count;
                                                            break;
                                                        }
                                                    }
                                                }
                                            }
                                            break;
                                    }

                                    output += temp + loading[i][j][0].ToString();
                                    total_weight += loading[i][j][1];

                                }
                                else
                                {
                                    output += "-";
                                }
                                if (j < 5)
                                {
                                    output += ", ";
                                }
                            }

                            switch (i)
                            {
                                case 0:
                                    rank = "st";
                                    if (total_weight > 0)
                                    {
                                        for (int z = 0; z < car_info.Count; z++)
                                        {
                                            if (car_info_list[z] == i)
                                            {
                                                car_info.RemoveAt(z);
                                                car_info_list.RemoveAt(z);
                                                z = car_info.Count;
                                                break;

                                            }
                                        }

                                    }

                                    break;

                                case 1:
                                    rank = "nd";
                                    if (total_weight > 0)
                                    {
                                        for (int z = 0; z < car_info.Count; z++)
                                        {
                                            if (car_info_list[z] == i)
                                            {
                                                car_info.RemoveAt(z);
                                                car_info_list.RemoveAt(z);
                                                z = car_info.Count;
                                                break;

                                            }
                                        }
                                    }

                                    break;

                                case 2:
                                    rank = "rd";
                                    if (total_weight > 0)
                                    {
                                        for (int z = 0; z < car_info.Count; z++)
                                        {
                                            if (car_info_list[z] == i)
                                            {
                                                car_info.RemoveAt(z);
                                                car_info_list.RemoveAt(z);
                                                z = car_info.Count;
                                                break;

                                            }
                                        }
                                    }

                                    break;

                                default:
                                    rank = "th";
                                    if (total_weight > 0)
                                    {
                                        for (int z = 0; z < car_info.Count; z++)
                                        {
                                            if (car_info_list[z] == i)
                                            {
                                                car_info.RemoveAt(z);
                                                car_info_list.RemoveAt(z);
                                                z = car_info.Count;
                                                break;

                                            }
                                        }
                                    }

                                    break;
                            }


                            Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                                weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4)");


                            csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + total_weight.ToString() + "/" +
                                weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                        }

                        csv_output.WriteLine("\n(1) 20l:" + w20l_container_list_overall.Count + " containers left");
                        List<List<double>> w20l_rest = new List<List<double>>();
                        List<List<double>> w20e_rest = new List<List<double>>();
                        List<List<double>> w40_rest = new List<List<double>>();

                        for (int i = 0; i <= w20l_container_list_overall.Count - 1; i++)
                        {
                            w20l_rest.Add(w20l_second[(int)w20l_container_list_overall[i] - 1]);
                            csv_output.Write("x" + w20l_container_list_overall[i] + ",");
                        }

                        csv_output.WriteLine();

                        csv_output.WriteLine("(2) 20e:" + w20e_container_list_overall.Count + " containers left");
                        for (int i = 0; i <= w20e_container_list_overall.Count - 1; i++)
                        {
                            w20e_rest.Add(w20e_second[(int)w20e_container_list_overall[i] - 1]);
                            csv_output.Write("v" + w20e_container_list_overall[i] + ",");
                        }

                        csv_output.WriteLine();

                        csv_output.WriteLine("(3) 40:" + w40_container_list_overall.Count + " containers left");
                        for (int i = 0; i <= w40_container_list_overall.Count - 1; i++)
                        {
                            w40_rest.Add(w40_second[(int)w40_container_list_overall[i] - 1]);
                            csv_output.Write("y" + w40_container_list_overall[i] + ",");
                        }

                        double total_time_first_stage = time1 + time2;
                        Console.WriteLine("+------------+\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                        " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));
                        Console.WriteLine("\n+---Result---+\n| Total used-time:" + (total_time_first_stage / 1000).ToString("#.####") + " seconds");
                        double utility_first_stage = (car_amount1 + car_amount * ob2) / car_amount_all;
                        Console.WriteLine("| Total utility:" + utility_first_stage + "\n+------------+");

                        csv_output.WriteLine("\ntime-used," + time2 / 1000 + ",,obj-value," + new_model.GetObjValue().ToString(".##"));
                        csv_output.WriteLine("-\nResult\ntotal-time(sec)," + total_time_first_stage / 1000 + ",,total utility," + utility_first_stage);

                        csv_output.WriteLine("-\nThird stage");
                        new_model.End();

                        car_amount = car_info.Count;
                        int car_for_u = car_amount; //car left

                        bool check = check_stage2(car_amount, w20l_rest, w20e_rest, w40_rest, hub_set);

                        new_model.End();
                        #endregion
                        #region third stage

                        if (check == true)
                        {
                            stack_amount = 2;
                            x_weights = new double[w20l_second.Count];

                            for (int i = 0; i < w20l_second.Count; i++)
                            {
                                x_weights[i] = w20l_second[i][0];
                            }

                            v_weights = new double[w20e_second.Count];

                            for (int i = 0; i < w20e_second.Count; i++)
                            {
                                v_weights[i] = w20e_second[i][0];
                            }

                            y_weights = new double[w40_second.Count];

                            for (int i = 0; i < w40_second.Count; i++)
                            {
                                y_weights[i] = w40_second[i][0];
                            }

                            hub_set = get_hub_sets(w20l_second, w20e_second, w40_second);

                            hub_amount = hub_set.Count;

                            x_amount = x_weights.Length;
                            v_amount = v_weights.Length;
                            y_amount = y_weights.Length;
                            car_amount = car_info_second.Count;

                            weight_limit = new double[car_amount];
                            weight_tolerence_factor = new double[car_amount];

                            for (int i = 0; i < car_amount; i++)
                            {
                                weight_limit[i] = car_info_second[i][0];
                                weight_tolerence_factor[i] = car_info_second[i][1];
                            }
                            //////////////////////////////////////////
                            Cplex new_model2 = new Cplex();

                            x_variables = new INumVar[x_weights.Length][];
                            v_variables = new INumVar[v_weights.Length][];
                            y_variables = new INumVar[y_weights.Length][][];
                            u_variables = new INumVar[hub_set.Count][];
                            z_variables = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                            t_variables = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                            for (int i = 0; i < x_weights.Length; i++)
                            {
                                x_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                            }

                            for (int i = 0; i < v_weights.Length; i++)
                            {
                                v_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                            }

                            for (int i = 0; i < y_weights.Length; i++)
                            {

                                y_variables[i] = new INumVar[car_amount][];

                                for (int j = 0; j < car_amount; j++)
                                {
                                    y_variables[i][j] = new_model2.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                                }

                            }

                            for (int i = 0; i < hub_set.Count; i++)
                            {
                                u_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                            }
                            double u2 = utility_upper_bound(car_for_u, car_amount_all, utility_first_stage, w20l_rest, w20e_rest, w40_rest, hub_set);

                            // declare the objective function

                            Console.WriteLine("[ 0/14]::Building objective function");

                            x_total_cost = new_model2.LinearNumExpr();
                            v_total_cost = new_model2.LinearNumExpr();
                            y_total_cost = new_model2.LinearNumExpr();

                            multiplier = 1 / (4 * (double)car_amount);

                            for (int i = 0; i < x_weights.Length; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    x_total_cost.AddTerm(multiplier, x_variables[i][j]);
                                }
                            }

                            for (int i = 0; i < v_weights.Length; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    v_total_cost.AddTerm(multiplier, v_variables[i][j]);
                                }
                            }

                            for (int i = 0; i < y_weights.Length; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    for (int k = 0; k < stack_amount; k++)
                                    {
                                        y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][k]);
                                    }

                                }
                            }

                            new_model2.AddMaximize(new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost));

                            new_model2.AddLe((new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost)), u2);
                            new_model2.AddGe((new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost)), utility_first_stage);

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint2a = new_model2.LinearNumExpr();

                                    for (int j = 0; j < x_amount; j++)
                                    {
                                        if (w20l_second[j][1] == hub_set[h])
                                        {
                                            constraint2a.AddTerm(1, x_variables[j][i]);
                                        }
                                    }

                                    constraint2a.AddTerm(-2, z_variables[i]);

                                    new_model2.AddLe(constraint2a, 0);

                                    constraint2a.Clear();

                                }
                            }
                            // constraint 2b

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint2b = new_model2.LinearNumExpr();

                                    for (int j = 0; j < x_amount; j++)
                                    {
                                        if (w20l_second[j][1] == hub_set[h])
                                        {
                                            constraint2b.AddTerm(1, x_variables[j][i]);
                                        }
                                    }

                                    constraint2b.AddTerm(-2, u_variables[h][i]);

                                    new_model2.AddLe(constraint2b, 0);

                                    constraint2b.Clear();

                                }
                            }

                            // constraint 2c

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint2b = new_model2.LinearNumExpr();

                                    for (int j = 0; j < x_amount; j++)
                                    {
                                        if (w20l_second[j][1] == hub_set[h])
                                        {
                                            constraint2b.AddTerm(1, x_variables[j][i]);
                                        }
                                    }

                                    constraint2b.AddTerm(-2, u_variables[h][i]);
                                    constraint2b.AddTerm(-2, z_variables[i]);

                                    new_model2.AddGe(constraint2b, -2);

                                    constraint2b.Clear();

                                }
                            }

                            // constraint 3

                            Console.WriteLine("[ 2/12]::Building constraint3");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < x_amount; i++)
                                {
                                    if (w20l_second[i][1] == hub_set[h])
                                    {
                                        ILinearNumExpr constraint3 = new_model2.LinearNumExpr();

                                        for (int j = 0; j < car_amount; j++)
                                        {
                                            constraint3.AddTerm(1, x_variables[i][j]);
                                        }

                                        new_model2.AddLe(constraint3, 1);

                                        constraint3.Clear();
                                    }
                                }
                            }

                            // constraint 4a 

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint4a = new_model2.LinearNumExpr();

                                    for (int j = 0; j < v_amount; j++)
                                    {
                                        if (w20e_second[j][1] == hub_set[h])
                                        {
                                            constraint4a.AddTerm(1, v_variables[j][i]);
                                        }
                                    }

                                    constraint4a.AddTerm(-2, t_variables[i]);

                                    new_model2.AddLe(constraint4a, 0);

                                    constraint4a.Clear();

                                }
                            }

                            // constraint 4b 

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint4b = new_model2.LinearNumExpr();

                                    for (int j = 0; j < v_amount; j++)
                                    {
                                        if (w20e_second[j][1] == hub_set[h])
                                        {
                                            constraint4b.AddTerm(1, v_variables[j][i]);
                                        }
                                    }

                                    constraint4b.AddTerm(-2, u_variables[h][i]);

                                    new_model2.AddLe(constraint4b, 0);

                                    constraint4b.Clear();

                                }
                            }

                            // constraint 4c

                            Console.WriteLine("[ 1/12]::Building constraint1");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint4c = new_model2.LinearNumExpr();

                                    for (int j = 0; j < v_amount; j++)
                                    {
                                        if (w20e_second[j][1] == hub_set[h])
                                        {
                                            constraint4c.AddTerm(1, v_variables[j][i]);
                                        }
                                    }

                                    constraint4c.AddTerm(-2, u_variables[h][i]);
                                    constraint4c.AddTerm(-2, t_variables[i]);

                                    new_model2.AddGe(constraint4c, -2);

                                    constraint4c.Clear();

                                }
                            }

                            // constraint 5

                            Console.WriteLine("[ 4/12]::Building constraint5");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < v_amount; i++)
                                {
                                    if (w20e_second[i][1] == hub_set[h])
                                    {
                                        ILinearNumExpr constraint5 = new_model2.LinearNumExpr();

                                        for (int j = 0; j < car_amount; j++)
                                        {
                                            constraint5.AddTerm(1, v_variables[i][j]);
                                        }


                                        new_model2.AddLe(constraint5, 1);

                                        constraint5.Clear();
                                    }
                                }
                            }
                            // constraint 6

                            Console.WriteLine("[ 4/14]::Building constraint4-1");

                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint6 = new_model2.LinearNumExpr();

                                constraint6.AddTerm(1, z_variables[i]);
                                constraint6.AddTerm(1, t_variables[i]);

                                new_model2.AddLe(constraint6, 1);

                                constraint6.Clear();
                            }


                            // constraint 7

                            Console.WriteLine("[ 8/14]::Building constraint7");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < y_amount; i++)
                                {
                                    if (w40_second[i][1] == hub_set[h])
                                    {
                                        ILinearNumExpr constraint7 = new_model2.LinearNumExpr();

                                        for (int j = 0; j < car_amount; j++)
                                        {
                                            for (int k = 0; k < stack_amount; k++)
                                            {
                                                constraint7.AddTerm(1, y_variables[i][j][k]);
                                            }
                                        }

                                        new_model2.AddLe(constraint7, 1);

                                        constraint7.Clear();
                                    }
                                }
                            }

                            // constraint 8 

                            Console.WriteLine("[ 9/14]::Building constraint8");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint8 = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint8.AddTerm(1, y_variables[j][i][0]);
                                        }
                                    }

                                    constraint8.AddTerm(-1, u_variables[h][i]);

                                    new_model2.AddLe(constraint8, 0);
                                }
                            }
                            // constraint 9a

                            Console.WriteLine("[10/14]::Building constraint11");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint9a = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint9a.AddTerm(1, y_variables[j][i][1]);
                                        }
                                    }

                                    constraint9a.AddTerm(1, z_variables[i]);

                                    new_model2.AddLe(constraint9a, 1);
                                }
                            }

                            // constraint 9b

                            Console.WriteLine("[10/14]::Building constraint11");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint9b = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {

                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint9b.AddTerm(1, y_variables[j][i][1]);
                                        }
                                    }

                                    constraint9b.AddTerm(-1, u_variables[h][i]);

                                    new_model2.AddLe(constraint9b, 0);
                                }
                            }

                            // constraint 10a

                            Console.WriteLine("[10/14]::Building constraint11");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint10a = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint10a.AddTerm(1, y_variables[j][i][1]);
                                        }
                                    }

                                    constraint10a.AddTerm(1, t_variables[i]);

                                    new_model2.AddLe(constraint10a, 1);
                                }
                            }

                            // constraint 10b

                            Console.WriteLine("[10/14]::Building constraint11");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint10b = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint10b.AddTerm(1, y_variables[j][i][1]);
                                        }
                                    }

                                    constraint10b.AddTerm(-1, u_variables[h][i]);

                                    new_model2.AddLe(constraint10b, 0);
                                }
                            }


                            // constraint 11

                            Console.WriteLine("[12/14]::Building constraint11");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint11 = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            for (int k = 0; k < stack_amount; k++)
                                            {
                                                constraint11.AddTerm(y_weights[j], y_variables[j][i][k]);
                                            }
                                        }
                                    }

                                    for (int j = 0; j < x_amount; j++)
                                    {
                                        if (w20l_second[j][1] == hub_set[h])
                                        {
                                            constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                                        }
                                    }

                                    for (int j = 0; j < v_amount; j++)
                                    {
                                        if (w20e_second[j][1] == hub_set[h])
                                        {
                                            constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                                        }
                                    }

                                    new_model2.AddLe(constraint11, weight_limit[i]);

                                    constraint11.Clear();
                                }
                            }

                            // constraint 12 

                            Console.WriteLine("[13/14]::Building constraint12");

                            for (int h = 0; h < hub_amount; h++)
                            {
                                for (int i = 0; i < car_amount; i++)
                                {
                                    ILinearNumExpr constraint12 = new_model2.LinearNumExpr();

                                    for (int j = 0; j < y_amount; j++)
                                    {
                                        if (w40_second[j][1] == hub_set[h])
                                        {
                                            constraint12.AddTerm(-1 * y_weights[j], y_variables[j][i][0]);
                                            constraint12.AddTerm(weight_tolerence_factor[i] * y_weights[j], y_variables[j][i][1]);
                                        }
                                    }

                                    for (int j = 0; j < x_amount; j++)
                                    {
                                        if (w20l_second[j][1] == hub_set[h])
                                        {
                                            constraint12.AddTerm(weight_tolerence_factor[i] * x_weights[j], x_variables[j][i]);
                                        }
                                    }

                                    for (int j = 0; j < v_amount; j++)
                                    {
                                        if (w20e_second[j][1] == hub_set[h])
                                        {
                                            constraint12.AddTerm(weight_tolerence_factor[i] * v_weights[j], v_variables[j][i]);
                                        }
                                    }

                                    new_model2.AddGe(constraint12, 0);

                                    constraint12.Clear();
                                }
                            }


                            //constraint 13

                            Console.WriteLine("[14/14]::Building constraint13");

                            for (int i = 0; i < car_amount; i++)
                            {
                                ILinearNumExpr constraint13 = new_model2.LinearNumExpr();

                                for (int h = 0; h < hub_amount; h++)
                                {
                                    constraint13.AddTerm(1, u_variables[h][i]);
                                }

                                new_model2.AddEq(constraint13, 1);

                                constraint13.Clear();
                            }


                            // model-solving

                            Console.WriteLine("\n> Start solving the model");

                            System.Diagnostics.Stopwatch gg = new System.Diagnostics.Stopwatch();

                            gg.Reset(); gg.Start();

                            new_model2.Solve();

                            gg.Stop();


                            Console.WriteLine("\nFinish solving, used time: " + (gg.Elapsed.TotalMilliseconds).ToString("0.####") +
                                " milliseconds,objective value=" + new_model2.GetObjValue().ToString("0.####"));
                            double time3 = gg.Elapsed.TotalMilliseconds;
                            double total_time = time1 + time2 + time3;
                            double ob3 = new_model2.GetObjValue();

                            x_result = new double[x_amount][];

                            for (int i = 0; i < x_amount; i++)
                            {
                                x_result[i] = new_model2.GetValues(x_variables[i]);
                            }

                            v_result = new double[v_amount][];

                            for (int i = 0; i < v_amount; i++)
                            {
                                v_result[i] = new_model2.GetValues(v_variables[i]);
                            }

                            y_result = new double[y_amount][][];

                            for (int i = 0; i < y_amount; i++)
                            {
                                y_result[i] = new double[car_amount][];

                                for (int j = 0; j < car_amount; j++)
                                {
                                    y_result[i][j] = new_model2.GetValues(y_variables[i][j]);
                                }
                            }

                            u_result = new double[hub_amount][];

                            for (int i = 0; i < hub_amount; i++)
                            {
                                u_result[i] = new_model2.GetValues(u_variables[i]);
                            }

                            // transform result into readable form

                            loading = new double[car_amount][][];

                            for (int i = 0; i < car_amount; i++)
                            {
                                loading[i] = new double[6][];

                                for (int j = 0; j < 6; j++)
                                {
                                    loading[i][j] = new double[2];
                                }
                            }

                            // record x-result

                            for (int i = 0; i < x_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (x_result[i][j] == 1)
                                    {
                                        if (loading[j][0][0] == 0)
                                        {
                                            loading[j][0][0] = i + 1;
                                            loading[j][0][1] = x_weights[i];
                                        }
                                        else
                                        {
                                            loading[j][1][0] = i + 1;
                                            loading[j][1][1] = x_weights[i];
                                        }
                                    }
                                }
                            }

                            // record v-result

                            for (int i = 0; i < v_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (v_result[i][j] == 1)
                                    {
                                        if (loading[j][2][0] == 0)
                                        {
                                            loading[j][2][0] = i + 1;
                                            loading[j][2][1] = v_weights[i];
                                        }
                                        else
                                        {
                                            loading[j][3][0] = i + 1;
                                            loading[j][3][1] = v_weights[i];
                                        }
                                    }
                                }
                            }

                            // record y-result

                            for (int i = 0; i < y_amount; i++)
                            {
                                for (int j = 0; j < car_amount; j++)
                                {
                                    if (y_result[i][j][1] == 1)
                                    {
                                        loading[j][4][0] = i + 1;
                                        loading[j][4][1] = y_weights[i];
                                    }
                                    if (y_result[i][j][0] == 1)
                                    {
                                        loading[j][5][0] = i + 1;
                                        loading[j][5][1] = y_weights[i];
                                    }
                                }
                            }

                            // output the result

                            csv_output.WriteLine("Car,20L,20L,20E,20E,40lower,40upper,,weight utility,space utility");

                            Console.WriteLine("\n+---result---+");

                            for (int i = 0; i < car_amount; i++)
                            {
                                string output = "", temp = "", rank = "";
                                double total_weight = 0;
                                double space_utility = 0;

                                for (int j = 0; j < 6; j++)
                                {
                                    if (loading[i][j][0] != 0)
                                    {
                                        switch (j)
                                        {
                                            case 0:
                                            case 1:
                                                temp = "x";
                                                space_utility++;
                                                break;
                                            case 2:
                                            case 3:
                                                temp = "v";
                                                space_utility++;
                                                break;
                                            case 4:
                                            case 5:
                                                temp = "y";
                                                space_utility += 2;
                                                break;
                                        }

                                        output += temp + loading[i][j][0].ToString();
                                        total_weight += loading[i][j][1];
                                    }
                                    else
                                    {
                                        output += "-";
                                    }
                                    if (j < 5)
                                    {
                                        output += ", ";
                                    }
                                }

                                switch (i)
                                {
                                    case 0:
                                        rank = "st";
                                        break;
                                    case 1:
                                        rank = "nd";
                                        break;
                                    case 2:
                                        rank = "rd";
                                        break;
                                    default:
                                        rank = "th";
                                        break;
                                }

                                string destination;

                                if (find_destination(u_result, i) != -1)
                                {
                                    destination = hub_set[find_destination(u_result, i)].ToString();
                                }
                                else
                                {
                                    destination = "none";
                                }

                                Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                                    weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4)");

                                csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + total_weight.ToString() + "/" +
                                     weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                            }
                            Console.WriteLine(u2);
                            csv_output.Write("upper-bound: ," + u2);
                            csv_output.Write("\ntime used(sec)," + (gg.Elapsed.TotalMilliseconds / 1000) + ",,obj-value," + new_model2.GetObjValue());
                            csv_output.Write("\n\ntotal time(sec):," + total_time / 1000 + ",,final utitility:," + ob3);
                            Console.WriteLine("\ntotal_time(sec)  " + total_time / 1000 + "\nutility:   " + ob3);
                            new_model2.End();

                        }
                        #endregion
                    }
                    csv_output.Close();

                } while (false);


                do
                {
                    #region multiple well car
                    int well_car_amount = 1;
                    Console.WriteLine("Overall process: new model");

                    // input the information

                    List<List<double>> w20l = new List<List<double>>();
                    List<List<double>> w20e = new List<List<double>>();
                    List<List<double>> w40 = new List<List<double>>();

                    string file_path = overall_file_path[a] + "\\w20l.csv";

                    if (file_path != "none")
                    {
                        StreamReader w20l_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w20l_file.ReadLine();

                        while ((data = w20l_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w20l.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                        }
                    }

                    file_path = overall_file_path[a] + "\\w20e.csv";

                    if (file_path != "none")
                    {
                        StreamReader w20e_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w20e_file.ReadLine();

                        while ((data = w20e_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w20e.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                        }
                    }

                    file_path = overall_file_path[a] + "\\w40.csv";

                    if (file_path != "none")
                    {
                        StreamReader w40_file = new StreamReader(file_path);

                        string[] values = null;
                        string data = w40_file.ReadLine();

                        while ((data = w40_file.ReadLine()) != null)
                        {
                            values = data.Split(',');
                            w40.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                        }
                    }

                    List<double> hub_set = get_hub_sets(w20l, w20e, w40);


                    double[] x_weights = new double[w20l.Count];

                    for (int i = 0; i < w20l.Count; i++)
                    {
                        x_weights[i] = w20l[i][0];
                    }

                    double[] v_weights = new double[w20e.Count];

                    for (int i = 0; i < w20e.Count; i++)
                    {
                        v_weights[i] = w20e[i][0];
                    }

                    double[] y_weights = new double[w40.Count];

                    for (int i = 0; i < w40.Count; i++)
                    {
                        y_weights[i] = w40[i][0];
                    }

                    int hub_amount = hub_set.Count;

                    file_path = overall_file_path[a] + "\\car.csv";

                    double[] weight_limit, weight_tolerence_factor;
                    int car_amount;

                    if (file_path.Length > 10)
                    {

                        StreamReader car_file = new StreamReader(file_path);
                        List<List<double>> car_info = new List<List<double>>();
                        string[] values = null;

                        string car_data = car_file.ReadLine();

                        while ((car_data = car_file.ReadLine()) != null)
                        {
                            values = car_data.Split(',');
                            car_info.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                        }

                        car_amount = car_info.Count;

                        weight_limit = new double[car_amount];
                        weight_tolerence_factor = new double[car_amount];

                        for (int i = 0; i < car_amount; i++)
                        {
                            weight_limit[i] = car_info[i][0];
                            weight_tolerence_factor[i] = car_info[i][1];
                        }

                    }
                    else
                    {

                        Console.Write("\n+---Set parameters---+\nSet the car's amount:\n> ");

                        car_amount = int.Parse(Console.ReadLine());

                        weight_limit = new double[car_amount];
                        weight_tolerence_factor = new double[car_amount];

                        Console.Write("\nSet the car's weight limit:\n> ");
                        weight_limit = initial_array(weight_limit, double.Parse(Console.ReadLine()));

                        Console.Write("\nSet the car's tolerence factor of (upper_weight/lower_weight)\n> ");
                        weight_tolerence_factor = initial_array(weight_tolerence_factor, double.Parse(Console.ReadLine()));

                    }
                    int stack_amount = 2;

                    string result_file_path = overall_file_path[a] + "\\n_well_car_new_model.csv";


                    // model

                    // declare variables etc.

                    double upper_bound = UB(car_amount, w20l, w20e, w40, hub_set);
                    Console.WriteLine("upper bounddddddddddddddd: " + upper_bound);

                    Console.WriteLine("\n> Start building the model");

                    Cplex new_model = new Cplex();

                    new_model.SetParam(Cplex.Param.Threads, 4);
                    //  new_model.SetParam(Cplex.Param.TimeLimit, max_cpu_time);

                    INumVar[][] x_variables = new INumVar[x_weights.Length][];
                    INumVar[][] v_variables = new INumVar[v_weights.Length][];
                    INumVar[][][] y_variables = new INumVar[y_weights.Length][][];
                    INumVar[][] u_variables = new INumVar[hub_set.Count][];
                    INumVar[] z_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    INumVar[] t_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                    for (int i = 0; i < x_weights.Length; i++)
                    {
                        x_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    for (int i = 0; i < v_weights.Length; i++)
                    {
                        v_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    for (int i = 0; i < y_weights.Length; i++)
                    {

                        y_variables[i] = new INumVar[car_amount][];

                        for (int j = 0; j < car_amount; j++)
                        {
                            y_variables[i][j] = new_model.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                        }

                    }

                    for (int i = 0; i < hub_set.Count; i++)
                    {
                        u_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                    // declare the objective function

                    Console.WriteLine("[ 0/14]::Building objective function");

                    ILinearNumExpr x_total_cost = new_model.LinearNumExpr();
                    ILinearNumExpr v_total_cost = new_model.LinearNumExpr();
                    ILinearNumExpr y_total_cost = new_model.LinearNumExpr();

                    double multiplier = 1 / (4 * (double)car_amount);

                    for (int i = 0; i < x_weights.Length; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            x_total_cost.AddTerm(multiplier, x_variables[i][j]);
                        }
                    }

                    for (int i = 0; i < v_weights.Length; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            v_total_cost.AddTerm(multiplier, v_variables[i][j]);
                        }
                    }

                    for (int i = 0; i < y_weights.Length; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            for (int k = 0; k < stack_amount; k++)
                            {
                                y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][k]);
                            }
                        }
                    }

                    new_model.AddMaximize(new_model.Sum(new_model.Sum(x_total_cost, v_total_cost), y_total_cost));

                    // declare constraints

                    int x_amount = x_weights.Length, v_amount = v_weights.Length, y_amount = y_weights.Length;

                    // new constraint added (calculate the upper bound)
                    new_model.AddLe((new_model.Sum(new_model.Sum(x_total_cost, v_total_cost), y_total_cost)), upper_bound);

                    #region constraint
                    // constraint 2a 

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2a.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2a.AddTerm(-2, z_variables[i]);

                            new_model.AddLe(constraint2a, 0);

                            constraint2a.Clear();

                        }
                    }
                    // constraint 2b 

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2b.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2b.AddTerm(-2, u_variables[h][i]);

                            new_model.AddLe(constraint2b, 0);

                            constraint2b.Clear();

                        }
                    }

                    // constraint 2c

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint2c.AddTerm(1, x_variables[j][i]);
                                }
                            }

                            constraint2c.AddTerm(-2, u_variables[h][i]);
                            constraint2c.AddTerm(-2, z_variables[i]);

                            new_model.AddGe(constraint2c, -2);

                            constraint2c.Clear();

                        }
                    }

                    // constraint3

                    Console.WriteLine("[ 2/12]::Building constraint3");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < x_amount; i++)
                        {
                            if (w20l[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    constraint3.AddTerm(1, x_variables[i][j]);
                                }

                                new_model.AddLe(constraint3, 1);

                                constraint3.Clear();
                            }
                        }
                    }

                    // constraint 4a 

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4a.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4a.AddTerm(-2, t_variables[i]);

                            new_model.AddLe(constraint4a, 0);

                            constraint4a.Clear();

                        }
                    }

                    // constraint 4b 

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4b.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4b.AddTerm(-2, u_variables[h][i]);

                            new_model.AddLe(constraint4b, 0);

                            constraint4b.Clear();

                        }
                    }

                    // constraint 4c

                    Console.WriteLine("[ 1/12]::Building constraint1");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint4c.AddTerm(1, v_variables[j][i]);
                                }
                            }

                            constraint4c.AddTerm(-2, u_variables[h][i]);
                            constraint4c.AddTerm(-2, t_variables[i]);

                            new_model.AddGe(constraint4c, -2);

                            constraint4c.Clear();

                        }
                    }

                    // constraint5

                    Console.WriteLine("[ 4/12]::Building constraint5");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < v_amount; i++)
                        {
                            if (w20e[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    constraint5.AddTerm(1, v_variables[i][j]);
                                }


                                new_model.AddLe(constraint5, 1);

                                constraint5.Clear();
                            }
                        }
                    }
                    // constraint6

                    Console.WriteLine("[ 4/14]::Building constraint4-1");

                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                        constraint6.AddTerm(1, z_variables[i]);
                        constraint6.AddTerm(1, t_variables[i]);

                        new_model.AddLe(constraint6, 1);

                        constraint6.Clear();
                    }


                    // constraint 7

                    Console.WriteLine("[ 8/14]::Building constraint7");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < y_amount; i++)
                        {
                            if (w40[i][1] == hub_set[h])
                            {
                                ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                                for (int j = 0; j < car_amount; j++)
                                {
                                    for (int k = 0; k < stack_amount; k++)
                                    {
                                        constraint7.AddTerm(1, y_variables[i][j][k]);
                                    }
                                }

                                new_model.AddLe(constraint7, 1);

                                constraint7.Clear();
                            }
                        }
                    }

                    // constraint 8 

                    Console.WriteLine("[ 9/14]::Building constraint8");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint8.AddTerm(1, y_variables[j][i][0]);
                                }
                            }

                            constraint8.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint8, 0);
                        }
                    }
                    // constraint 9a

                    Console.WriteLine("[10/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint9a.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint9a.AddTerm(1, z_variables[i]);

                            new_model.AddLe(constraint9a, 1);
                        }
                    }

                    // constraint 9b

                    Console.WriteLine("[10/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {

                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint9b.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint9b.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint9b, 0);
                        }
                    }

                    // constraint 10a

                    Console.WriteLine("[10/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint10a.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint10a.AddTerm(1, t_variables[i]);

                            new_model.AddLe(constraint10a, 1);
                        }
                    }

                    // constraint 10b

                    Console.WriteLine("[10/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint10b.AddTerm(1, y_variables[j][i][1]);
                                }
                            }

                            constraint10b.AddTerm(-1, u_variables[h][i]);

                            new_model.AddLe(constraint10b, 0);
                        }
                    }


                    // constraint 11

                    Console.WriteLine("[12/14]::Building constraint11");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    for (int k = 0; k < stack_amount; k++)
                                    {
                                        constraint11.AddTerm(y_weights[j], y_variables[j][i][k]);
                                    }
                                }
                            }

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                                }
                            }

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                                }
                            }

                            new_model.AddLe(constraint11, weight_limit[i]);

                            constraint11.Clear();
                        }
                    }

                    // constraint 12 

                    Console.WriteLine("[13/14]::Building constraint12");

                    for (int h = 0; h < hub_amount; h++)
                    {
                        for (int i = 0; i < car_amount; i++)
                        {
                            ILinearNumExpr constraint12 = new_model.LinearNumExpr();

                            for (int j = 0; j < y_amount; j++)
                            {
                                if (w40[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(-1 * y_weights[j], y_variables[j][i][0]);
                                    constraint12.AddTerm(weight_tolerence_factor[i] * y_weights[j], y_variables[j][i][1]);
                                }
                            }

                            for (int j = 0; j < x_amount; j++)
                            {
                                if (w20l[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(weight_tolerence_factor[i] * x_weights[j], x_variables[j][i]);
                                }
                            }

                            for (int j = 0; j < v_amount; j++)
                            {
                                if (w20e[j][1] == hub_set[h])
                                {
                                    constraint12.AddTerm(weight_tolerence_factor[i] * v_weights[j], v_variables[j][i]);
                                }
                            }

                            new_model.AddGe(constraint12, 0);

                            constraint12.Clear();
                        }
                    }


                    //constraint 13

                    Console.WriteLine("[14/14]::Building constraint13");

                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                        for (int h = 0; h < hub_amount; h++)
                        {
                            constraint13.AddTerm(1, u_variables[h][i]);
                        }

                        new_model.AddEq(constraint13, 1);

                        constraint13.Clear();
                    }

                    int total_big_car = car_amount / 5;

                    int initial = 0;



                    Console.WriteLine("new well car constraint");

                    for (int k = 0; k < total_big_car; k++)
                    {
                        for (int h = 0; h < hub_amount; h++)
                        {
                            ILinearNumExpr constraint14 = new_model.LinearNumExpr();

                            for (int i = initial; i < initial + well_car_amount; i++)
                            {

                                constraint14.AddTerm(1, u_variables[h][i]);

                            }

                            constraint14.AddTerm(-well_car_amount, u_variables[h][initial]);

                            new_model.AddEq(constraint14, 0);

                            constraint14.Clear();


                        }
                        initial += well_car_amount;
                    }


                    #endregion

                    // model-solving

                    Console.WriteLine("\n> Start solving the model");

                    System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

                    sw.Reset(); sw.Start();

                    new_model.Solve();

                    sw.Stop();

                    // get the result variable values

                    Console.WriteLine("\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                        " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));

                    double[][] x_result = new double[x_amount][];

                    for (int i = 0; i < x_amount; i++)
                    {
                        x_result[i] = new_model.GetValues(x_variables[i]);
                    }

                    double[][] v_result = new double[v_amount][];

                    for (int i = 0; i < v_amount; i++)
                    {
                        v_result[i] = new_model.GetValues(v_variables[i]);
                    }

                    double[][][] y_result = new double[y_amount][][];

                    for (int i = 0; i < y_amount; i++)
                    {
                        y_result[i] = new double[car_amount][];

                        for (int j = 0; j < car_amount; j++)
                        {
                            y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                        }
                    }

                    double[][] u_result = new double[hub_amount][];

                    for (int i = 0; i < hub_amount; i++)
                    {
                        u_result[i] = new_model.GetValues(u_variables[i]);
                    }

                    // transform result into readable form

                    double[][][] loading = new double[car_amount][][];

                    for (int i = 0; i < car_amount; i++)
                    {
                        loading[i] = new double[6][];

                        for (int j = 0; j < 6; j++)
                        {
                            loading[i][j] = new double[2];
                        }
                    }

                    // record x-result

                    for (int i = 0; i < x_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (x_result[i][j] == 1)
                            {
                                if (loading[j][0][0] == 0)
                                {
                                    loading[j][0][0] = i + 1;
                                    loading[j][0][1] = x_weights[i];
                                }
                                else
                                {
                                    loading[j][1][0] = i + 1;
                                    loading[j][1][1] = x_weights[i];
                                }
                            }
                        }
                    }

                    // record v-result

                    for (int i = 0; i < v_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (v_result[i][j] == 1)
                            {
                                if (loading[j][2][0] == 0)
                                {
                                    loading[j][2][0] = i + 1;
                                    loading[j][2][1] = v_weights[i];
                                }
                                else
                                {
                                    loading[j][3][0] = i + 1;
                                    loading[j][3][1] = v_weights[i];
                                }
                            }
                        }
                    }

                    // record y-result

                    for (int i = 0; i < y_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (y_result[i][j][1] == 1)
                            {
                                loading[j][4][0] = i + 1;
                                loading[j][4][1] = y_weights[i];
                            }
                            if (y_result[i][j][0] == 1)
                            {
                                loading[j][5][0] = i + 1;
                                loading[j][5][1] = y_weights[i];
                            }
                        }
                    }

                    // output the result

                    StreamWriter csv_output = new StreamWriter(result_file_path);
                    csv_output.WriteLine(",20L,20L,20E,20E,40lower,40upper,,destination,weight utility,space utility");

                    Console.WriteLine("\n+---result---+");

                    for (int i = 0; i < car_amount; i++)
                    {
                        string output = "", temp = "", rank = "";
                        double total_weight = 0;
                        double space_utility = 0;

                        for (int j = 0; j < 6; j++)
                        {
                            if (loading[i][j][0] != 0)
                            {
                                switch (j)
                                {
                                    case 0:
                                    case 1:
                                        temp = "x";
                                        space_utility++;
                                        break;
                                    case 2:
                                    case 3:
                                        temp = "v";
                                        space_utility++;
                                        break;
                                    case 4:
                                    case 5:
                                        temp = "y";
                                        space_utility += 2;
                                        break;
                                }

                                output += temp + loading[i][j][0].ToString();
                                total_weight += loading[i][j][1];
                            }
                            else
                            {
                                output += "-";
                            }
                            if (j < 5)
                            {
                                output += ", ";
                            }
                        }

                        switch (i)
                        {
                            case 0:
                                rank = "st";
                                break;
                            case 1:
                                rank = "nd";
                                break;
                            case 2:
                                rank = "rd";
                                break;
                            default:
                                rank = "th";
                                break;
                        }

                        string destination;

                        if (find_destination(u_result, i) != -1)
                        {
                            destination = hub_set[find_destination(u_result, i)].ToString();
                        }
                        else
                        {
                            destination = "none";
                        }

                        Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                            weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4), destination: hub[" +
                            destination + "]");

                        csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + destination + "," + total_weight.ToString() + "/" +
                            weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                    }

                    csv_output.Write("\nupper bound," + upper_bound + "\ntime used(sec)," + (sw.Elapsed.TotalMilliseconds / 1000) + ",,obj-value," + new_model.GetObjValue());

                    csv_output.Close();

                    new_model.End();
                    #endregion

                } while (false);
            }

        }


        static double[] initial_array(double[] obj_array, double value)
        {

            double[] result = new double[obj_array.Length];

            for (int i = 0; i < obj_array.Length; i++)
            {
                result[i] = value;
            }

            return result;

        }

        static double[] array_to_double(string[] obj_array)
        {

            double[] result = new double[obj_array.Length];

            for (int i = 0; i < obj_array.Length; i++)
            {
                result[i] = double.Parse(obj_array[i]);
            }

            return result;

        }

        static List<double> get_hub_sets(List<List<double>> list1, List<List<double>> list2, List<List<double>> list3)
        {

            List<double> result = new List<double>();

            for (int i = 0; i < list1.Count; i++)
            {
                if (!whether_in_list(result, list1[i][1]))
                {
                    result.Add(list1[i][1]);
                }
            }

            for (int i = 0; i < list2.Count; i++)
            {
                if (!whether_in_list(result, list2[i][1]))
                {
                    result.Add(list2[i][1]);
                }
            }

            for (int i = 0; i < list3.Count; i++)
            {
                if (!whether_in_list(result, list3[i][1]))
                {
                    result.Add(list3[i][1]);
                }
            }

            return result;

        }

        static bool whether_in_list(List<double> objective, double value)
        {

            bool result = false;

            for (int i = 0; i < objective.Count; i++)
            {
                if (objective[i] == value)
                {
                    result = true;
                }
            }

            return result;

        }

        static int find_destination(double[][] objective, int car_index)
        {

            int result = -1;

            for (int i = 0; i < objective.Length; i++)
            {
                if (objective[i][car_index] == 1)
                {
                    result = i;
                }
            }


            return result;

        }
        static List<double> container_amount(List<List<double>> container_list)
        {
            List<double> result = new List<double>();

            for (int i = 0; i <= container_list.Count - 1; i++)
            {
                result.Add(i + 1);
            }

            return result;
        }
        static bool check_stage2(int car_amount, List<List<double>> w20l, List<List<double>> w20e, List<List<double>> w40, List<double> hub_set) // 如果剩餘的貨櫃是去相同hub
        {
            bool result = false;

            if (car_amount > 0) //還有車車
            {
                if (w20l.Count >= 2 && same_hub_check(w20l, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
                if (w20e.Count >= 2 && same_hub_check(w20e, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
                if (w40.Count >= 1 && same_hub_check(w40, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
            }
        output:
            return result;

        }
        static bool same_hub_check(List<List<double>> container, List<double> hub_set)
        {
            bool result = false;
            int[] count = new int[hub_set.Count];

            for (int k = 0; k < hub_set.Count; k++)
            {
                for (int i = 0; i < container.Count; i++)
                {
                    if (container[i][1] == k + 1)
                    {
                        count[k]++;
                    }
                }
            }
            for (int j = 0; j < count.Length; j++)
            {
                if (count[j] > 1)
                {
                    result = true;
                    break;
                }
            }


            return result;
        }
        static double utility_upper_bound(int car_amount, int car_amount_all, double utility_first_stage, List<List<double>> w20l, List<List<double>> w20e, List<List<double>> w40, List<double> hub_set)
        {
            double utility_upper_bound = utility_first_stage;
            int a = 0, b = 0, c = 0;
            int[] count_a = count_pair(w20l, hub_set);
            if (car_amount > 0) //still have cars left
            {
                for (int i = 0; i < count_a.Length; i++)
                {
                    if (count_a[i] >= 2)
                    {
                        a = (count_a[i] / 2);
                        Console.WriteLine("a " + a);
                        utility_upper_bound = utility_upper_bound + (a * 0.5 / car_amount_all);
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }

            if (car_amount > 0)
            {
                int[] count_b = count_pair(w20e, hub_set);
                for (int i = 0; i < count_b.Length; i++)
                {
                    if (count_b[i] >= 2)
                    {
                        b = (count_b[i] / 2);
                        utility_upper_bound += b * 0.5 / car_amount_all;
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }

            int[] count_c = count_pair(w40, hub_set);
            if (car_amount > 0)
            {
                for (int i = 0; i < count_c.Length; i++)
                {
                    if (count_c[i] >= 1)
                    {
                        c = (count_c[i]);
                        utility_upper_bound += c * 0.5 / car_amount_all;
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }
            return utility_upper_bound;

        }
        static int[] count_pair(List<List<double>> container, List<double> hub_set)
        {
            int[] count_pair = new int[hub_set.Count];

            for (int k = 0; k < hub_set.Count; k++)
            {
                for (int i = 0; i < container.Count; i++)
                {
                    if (container[i][1] == k + 1)
                    {
                        count_pair[k]++;
                    }
                }
            }
            return count_pair;

        }

        static double UB(int car_amount, List<List<double>> w20l, List<List<double>> w20e, List<List<double>> w40, List<double> hub_set)
        {
            double UB = 0;
            int big_car_amount = car_amount / 1;
            int big_car_remain = car_amount % 1;
            int temp_big_car = 0;

            int[] w20l_count = new int[hub_set.Count];
            int[] w20e_count = new int[hub_set.Count];
            int[] w40_count = new int[hub_set.Count];


            for (int i = 0; i < w20l.Count; i++)
            {
                w20l_count[hub_set.IndexOf(w20l[i][1])]++;
            }


            for (int i = 0; i < w20e.Count; i++)
            {
                w20e_count[hub_set.IndexOf(w20e[i][1])]++;
            }


            for (int i = 0; i < w40.Count; i++)
            {
                w40_count[hub_set.IndexOf(w40[i][1])]++;
            }

            List<List<int>> required_car = new List<List<int>>();

            for (int i = 0; i < hub_set.Count; i++)
            {
                int w20_required = (int)(w20l_count[i] / 2) + (int)(w20e_count[i] / 2);

                if (w20_required >= w40_count[i])
                {
                    required_car.Add(new List<int> { w20_required, w40_count[i], w20_required - w40_count[i] });
                }
                else
                {
                    int remain_car = (w40_count[i] - w20_required) % 2;
                    int full_car = (int)((w40_count[i] - w20_required) / 2) + w20_required;
                    required_car.Add(new List<int> { full_car + remain_car, full_car, remain_car });
                }

            }


            for (int i = 0; i < required_car.Count; i++)
            {
                if (required_car[i][1] / 1 >= 1)
                {
                    int slack_amount = Math.Min(required_car[i][1] / 1, big_car_amount);

                    /*temp_big_car+= required_car[i][1] / 5;
                    big_car_amount-= required_car[i][1] / 5;*/

                    temp_big_car += slack_amount;
                    big_car_amount -= slack_amount;

                    required_car[i][1] -= 1 * slack_amount;
                    required_car[i][0] -= 1 * slack_amount;

                    if (big_car_amount == 0)
                    {
                        break;
                    }
                }
            }

            double u_max = 0;
            double u_second_total = 0;
            int index_max = 0;

            for (int j = big_car_amount; j > 0; j--)
            {
                for (int i = 0; i < required_car.Count; i++)
                {
                    int slack_amount = Math.Min(1 - required_car[i][1], required_car[i][2]);
                    double temp_max = (required_car[i][1] + slack_amount * 0.5) / 1;


                    if (temp_max > u_max)
                    {
                        u_max = temp_max;
                        index_max = i;
                    }
                }

                int a = 1 - required_car[index_max][1];
                required_car[index_max][1] = 0;
                required_car[index_max][2] -= a;

                u_second_total += u_max;
                u_max = 0;
            }


            int u_first = 0;
            int u_second = 0;

            for (int i = 0; i < required_car.Count; i++)
            {
                u_first += required_car[i][1];
            }

            if (u_first >= big_car_remain)
            {
                UB = (1 * temp_big_car + 1 * u_second_total + big_car_remain) / car_amount;
            }
            else
            {
                big_car_remain -= u_first;

                for (int i = 0; i < required_car.Count; i++)
                {
                    u_second += required_car[i][2];
                }

                if (u_second >= big_car_remain)
                {
                    UB = (1 * temp_big_car + 1 * u_second_total + u_first + 0.5 * big_car_remain) / car_amount;
                    //UB += big_car_remain / car_amount;
                }
                else
                {
                    UB = (1 * temp_big_car + 1 * u_second_total + u_first + 0.5 * u_second) / car_amount;
                }


            }
            return UB;




        }
    }
}
