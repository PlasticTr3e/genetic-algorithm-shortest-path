from config.config_parser import parser
from utils.obstacle_generator import generate_obstacles
from utils.path_point_generator import generate_path_points
from utils.plotter import plot_fitness_history
from genetic_algorithm import start, path_overlaps_obstacle

obstacles = []
path_points = []
path_validity = dict()

EXPERIMENTS = [
    # {
    #     'population_size': '10',
    #     'top_percentage': '0.80',   
    #     'mutation_probability': '0.1',  
    #     'crossover_split_random': 'false',
    #     'crossover_split_size': '0.5',
    #     'max_generations': '30',
    #     'label': 'Exp 1 : Pop:10 Top:0.8 Mut:0.1',
    #     'color': 'red',
    # },
    # {
    #     'population_size': '50',
    #     'top_percentage': '0.40',
    #     'mutation_probability': '0.3',  
    #     'crossover_split_random': 'false',
    #     'crossover_split_size': '0.5',
    #     'max_generations': '30',
    #     'label': 'Exp 2 : Pop:50 Top:0.4 Mut:0.3',
    #     'color': 'blue',
    # },
    {
    
        'population_size': '50',
        'top_percentage': '0.20',   
        'mutation_probability': '0.3',
        'crossover_split_random': 'true',  
        'crossover_split_size': '0.5',
        'max_generations': '15',
        'label': 'Exp 3 : Pop:150 Top:0.2 Mut:0.3 CrossRandom',
        'color': 'green',
    },
]


def main():
    _init_obstacles()
    _init_path_points()
    _init_path_validity()

    all_results = []

    for exp in EXPERIMENTS:
        print(f"\n{'='*60}")
        print(f"Running {exp['label']}")
        print(f"{'='*60}")
        for key in ('population_size', 'top_percentage', 'mutation_probability',
                    'crossover_split_random', 'crossover_split_size', 'max_generations'):
            parser['Genetic Algorithm'][key] = exp[key]

        best_history, avg_history, best_chromosome, best_length, max_angle_best = start(
            obstacles, path_points, path_validity, show_animation=False
        )

        print(f"Max Turn Angle (best) : {max_angle_best:.2f}")
        print(f"Constraint Limit      : {float(parser['Genetic Algorithm']['max_turn_angle']):.2f}")
        print(f"Constraint Satisfied  : {max_angle_best <= float(parser['Genetic Algorithm']['max_turn_angle'])}")

        print(f"Best Genome/Solution for {exp['label']} {best_chromosome}")
        print(f"Best Path Length : {best_length:.4f}")
        print(f"Best Fitness     : {1/best_length:.6f}")

def _init_path_points():
    if parser['Path Points'].getboolean('generate_randomly'):
        generate_path_points(path_points, obstacles)
    else:
        for element in eval(parser['Hardcoded Path Points']['path_points']):
            path_points.append(element)

def _init_obstacles():
    if parser['Obstacles'].getboolean('generate_randomly'):
        number_of_obstacles = int(parser['Obstacles']['number_of_obstacles'])
        generate_obstacles(obstacles, number_of_obstacles)
    else:
        for i in range(int(parser['Hardcoded Obstacles']['number_of_hardcoded_obstacles'])):
            obstacle = eval(parser['Hardcoded Obstacles'][f"obstacle_{i+1}"])
            obstacles.append(obstacle)

def _init_path_validity():
    for i, path_point_start in enumerate(path_points):
        if path_point_start not in path_validity:
            path_validity[path_point_start] = [True] * len(path_points)
        for j, path_point_end in enumerate(path_points):
            if path_point_end not in path_validity:
                path_validity[path_point_end] = [True] * len(path_points)
            if path_overlaps_obstacle(path_point_start, path_point_end, obstacles):
                path_validity[path_point_start][j] = False
                path_validity[path_point_end][i] = False

if __name__ == '__main__':
    main()