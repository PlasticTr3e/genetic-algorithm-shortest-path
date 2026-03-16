import math
from config.config_parser import parser
from random import randint

from shapely.geometry import Polygon, LineString

from utils.plotter import plot, plot_fitness_history

def start(obstacles, path_points, path_validity, show_animation=True):

    population = _generate_population(path_points, obstacles, path_validity)

    for i, chromosome in enumerate(population, start=1):
        chromosome_array = [int(gene) for gene in chromosome]
        length = _calculate_path_length(chromosome, path_points)
        print(f"Chromosome {i}: {chromosome_array} | Length: {length}")

    path_lengths = []
    for chromosome in population:
        path_lengths.append(_calculate_path_length(chromosome, path_points))

    best_fitness_history = [1 / min(path_lengths)]
    avg_fitness_history = [sum(1 / length for length in path_lengths) / len(path_lengths)]

    if show_animation:
        plot(obstacles, path_points, population, path_lengths, 1, False)

    generations = int(parser['Genetic Algorithm']['max_generations'])
    
    for gen in range(generations - 1):
        new_population = []
        path_lengths.clear()

        fitness_list = _sort_by_fitness(population, path_points)

        for chromosome in population:
            while True:
                parent1 = _choose_random_parent(fitness_list)
                parent2 = _choose_random_parent(fitness_list)

                child = _crossover(parent1, parent2)

                if randint(1, 10) <= 10 * float(parser['Genetic Algorithm']['mutation_probability']):
                    child = _mutation(child)

                if _chromosome_valid(child, obstacles, path_points):
                    break
            
            path_lengths.append(_calculate_path_length(child, path_points))
            new_population.append(child)

        population = new_population

        best_fitness_history.append(1 / min(path_lengths))
        avg_fitness_history.append(sum(1 / length for length in path_lengths) / len(path_lengths))

        if show_animation:
            plot(obstacles, path_points, new_population, path_lengths, (gen + 2), last_gen=True if gen == generations - 2 else False)

    best_index = population.index(min(population, key=lambda c: _calculate_path_length(c, path_points)))
    best_chromosome = population[best_index]
    best_length = _calculate_path_length(best_chromosome, path_points)
    best_chromosome_array = [int(gene) for gene in best_chromosome]

    max_angle_best = 0.0
    visited_points = [path_points[i] for i, gene in enumerate(best_chromosome) if gene == '1']
    if len(visited_points) >= 3:
        for i in range(1, len(visited_points) - 1):
            angle = _turn_angle_degrees(visited_points[i - 1], visited_points[i], visited_points[i + 1])
            max_angle_best = max(max_angle_best, angle)

    if not show_animation:
        plot(obstacles, path_points, [best_chromosome], [best_length], generations, last_gen=True)

    return best_fitness_history, avg_fitness_history, best_chromosome_array, best_length, max_angle_best

def _mutation(chromosome):
    index = randint(1, len(chromosome) - 2) # we won't mutate source and goal genes

    chromosome = list(chromosome)
    chromosome[index] = '1' if  chromosome[index] == '0' else '0'

    return ''.join(chromosome)

def _fitness(chromosome, path_points):
    max_turn_angle = float(parser['Genetic Algorithm']['max_turn_angle'])

    if _violates_turn_constraint(chromosome, path_points, max_turn_angle):
        return 0

    length = _calculate_path_length(chromosome, path_points)
    fitness = 1 / length if length != 0 else 0

    return fitness

def _sort_by_fitness(population, path_points):
    fitness_list = []

    for chromosome in population:
        chromosome_to_fitness = (chromosome, _fitness(chromosome, path_points))
        fitness_list.append(chromosome_to_fitness)

    fitness_list.sort(reverse=True, key=lambda tuple: tuple[1])

    return fitness_list

def _choose_random_parent(fitness_list):
    till_index = len(fitness_list) * float(parser['Genetic Algorithm']['top_percentage'])
    till_index = math.floor(till_index)

    parent_to_fitness = fitness_list[randint(0, till_index)]

    return parent_to_fitness[0]

def _crossover(parent1, parent2):

    if parser['Genetic Algorithm'].getboolean('crossover_split_random'):
        split_size = randint(0, len(parent1))

    else:
        fraction = float(parser['Genetic Algorithm']['crossover_split_size'])
        split_size = math.floor(fraction * len(parent1))

    return ''.join([parent1[:split_size], parent2[split_size:]])

def _generate_population(path_points, obstacles, path_validity):

    population_size = int(parser['Genetic Algorithm']['population_size'])

    population = []
    print('Generating initial population, please wait ....')
    for i in range(population_size):
        while True:
            chromosome = _generate_chromosome(path_points, path_validity)
            if chromosome:
                break
            
        population.append(chromosome)

    print('Successfully created initial population')
    print('Simulating genetic algorithm for path planning .... (Press Ctrl+C to stop)')
    return population

def _generate_chromosome(path_points, path_validity):

    chromosome = '1' # source is always visited
    previous_path_point = path_points[0] # keep track of the previous path point that was 1
    
    for i in range(1, len(path_points)):
        path_point = path_points[i]

        if i == (len(path_points) - 1) and not path_validity[previous_path_point][i]:
            return False

        if path_validity[previous_path_point][i]:

            if i == (len(path_points) - 1):
                gene = '1'
            else:
                gene = '0' if randint(1, 10) > 5 else '1'

            if gene == '1':
                previous_path_point = path_point
            
            chromosome += gene

        else:
            chromosome += '0'

    return chromosome

def _chromosome_valid(chromosome, obstacles, path_points):
    path_point_1, path_point_2 = (), ()

    for i, gene in enumerate(chromosome):
        if gene == '1':

            if not path_point_1:
                path_point_1 = path_points[i] 
            else:
                path_point_2 = path_points[i]

            if path_point_1 and path_point_2:

                if path_overlaps_obstacle(path_point_1, path_point_2, obstacles):
                    return False

                path_point_1 = path_point_2
                path_point_2 = ()
    
    return True

def path_overlaps_obstacle(path_point_1, path_point_2, obstacles):
    path = LineString([path_point_1, path_point_2])

    for obstacle in obstacles:

        obstacle = Polygon(obstacle)
        if path.intersects(obstacle):
            return True

    return False


def _calculate_path_length(chromosome, path_points):
    path_point_1, path_point_2 = (), ()
    length = 0

    for i, gene in enumerate(chromosome):
        if gene == '1':
            last_path_point = path_points[i]

            if not path_point_1:
                path_point_1 = path_points[i] 
            else:
                path_point_2 = path_points[i]

            if path_point_1 and path_point_2:

                length += _distance(path_point_1, path_point_2)

                path_point_1 = path_point_2
                path_point_2 = ()

    return length

def _distance(path_point_1, path_point_2):
    return math.sqrt( (path_point_2[0] - path_point_1[0])**2 + (path_point_2[1] - path_point_1[1])**2 )


# Aturan baru vatas sudut belok maksimum.
def _turn_angle_degrees(p1, p2, p3):
    v1x, v1y = p1[0] - p2[0], p1[1] - p2[1]
    v2x, v2y = p3[0] - p2[0], p3[1] - p2[1]

    norm1 = math.sqrt(v1x**2 + v1y**2)
    norm2 = math.sqrt(v2x**2 + v2y**2)

    if norm1 == 0 or norm2 == 0:
        return 0

    cos_theta = (v1x * v2x + v1y * v2y) / (norm1 * norm2)
    cos_theta = max(-1.0, min(1.0, cos_theta))

    return math.degrees(math.acos(cos_theta))

def _violates_turn_constraint(chromosome, path_points, max_turn_angle):
    visited_points = [path_points[i] for i, gene in enumerate(chromosome) if gene == '1']
    if len(visited_points) < 3:
        return False

    for i in range(1, len(visited_points) - 1):
        angle = _turn_angle_degrees(
            visited_points[i - 1],
            visited_points[i],
            visited_points[i + 1]
        )
        if angle > max_turn_angle:
            return True

    return False