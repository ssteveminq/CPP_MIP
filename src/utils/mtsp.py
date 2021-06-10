import numpy as np
import random
import math
import matplotlib.pyplot as plt
'''
Genetical path finding
Finds locally best ways from L service centers with [M0, M1, ..., ML] engineers
through atms_number ATMs and back to their service center
'''

generations = 1500       # population's generations
mut_1_prob = 0.4         # prob of replacing together two atms in combined path
mut_2_prob = 0.6      # prob of reversing the sublist in combined path
mut_3_prob = 0.8     # probability of changing the length of paths for engineers
two_opt_search = False  # better convergence, lower speed for large quantity of atms


# def plot_paths(paths, axes=None):
    # plt.clf()
    # plt.title('Best path overall')
    # for v in range(service_centers):
        # plt.scatter(points_locations[v, 0], points_locations[v, 1], c='r')
    # for v in range(atms_number):
        # plt.scatter(points_locations[v+service_centers, 0], points_locations[v+service_centers, 1], c='b')
    # for v in range(len(paths)):
        # if len(paths[v]) != 0:
            # path_locations = points_locations[service_centers:]
            # path_locations = path_locations[np.array(paths[v])]
            # path_locations = np.vstack((points_locations[engineers[v]], path_locations))
            # path_locations = np.vstack((path_locations, points_locations[engineers[v]]))
            # plt.plot(path_locations[:, 0], path_locations[:, 1])
    # plt.show()
    # plt.pause(0.0001)


class tsp_manager:
    def __init__(self, num_agents, points, starting_points):
    # def __init__(self):
        # self.atms_number = 20         # ATM quantity
        # self.service_centers = 2                # service centers quantity
        self.atms_number = len(points)         # ATM quantity
        self.service_centers = num_agents     # service centers quantity
        self.points=points

        self.points_locations = np.zeros((self.service_centers+self.atms_number, 2))
        if num_agents>1:
            for i in range(num_agents):
                self.points_locations[i,0]=starting_points[i][0]
                self.points_locations[i,1]=starting_points[i][1]
        else:
            self.points_locations[0,0]=starting_points[0]
            self.points_locations[0,1]=starting_points[1]

        for j in range(self.atms_number):
            self.points_locations[j+num_agents,0]=points[j][0]
            self.points_locations[j+num_agents,1]=points[j][1]

        
        self.velocity = 100             # 100 / hour
        self.repair_time = 0         # 0.5 hour
        self.max_engi = 3              # maximum number of engineers in one service center
        #genetic parameters
        self.population_size = 50    # population size (even number!)
        self.generations = 1500       # population's self.generations
        self.mut_1_prob = 0.4         # prob of replacing together two atms in combined path
        self.mut_2_prob = 0.6      # prob of reversing the sublist in combined path
        self.mut_3_prob = 0.8     # probability of changing the length of paths for engineers
        self.two_opt_search = False  # better convergence, lower speed for large quantity of atms
        self.engineers = []
        # self.points_location=np.resize(5,2)
        

    def fitness_pop(self, population):
        fitness_result = np.zeros(len(population))
        for i in range(len(fitness_result)):
            fitness_result[i] = self.fitness(population[i])
        return fitness_result

    def fitness(self,creature):
        sum_dist = np.zeros(len(creature))
        for j in range(len(creature)):
            mat_path = np.zeros((self.dist.shape[0], self.dist.shape[1]))
            path = creature[j]
            if len(path) != 0:
                for v in range(len(path)):
                    if v == 0:
                        mat_path[self.engineers[j], path[v]] = 1
                    else:
                        mat_path[path[v - 1] + self.service_centers, path[v]] = 1
                mat_path = mat_path * self.dist
                sum_dist[j] = (np.sum(mat_path) + self.dist[self.engineers[j], path[-1]]) / self.velocity + self.repair_time * len(path)
        return np.max(sum_dist)

    def birth_prob(self, fitness_result):
        birth_prob = np.abs(fitness_result - np.max(fitness_result))
        birth_prob = birth_prob / np.sum(birth_prob)
        return birth_prob

    def mutate(self, creat, engi):
        pnt_1 = random.randint(0, len(creat)-1)
        pnt_2 = random.randint(0, len(creat)-1)
        if random.random() < self.mut_1_prob:
            creat[pnt_1], creat[pnt_2] = creat[pnt_2], creat[pnt_1]
        if random.random() < self.mut_2_prob and pnt_1 != pnt_2:
            if pnt_1 > pnt_2:
                pnt_1, pnt_2 = pnt_2, pnt_1
            creat[pnt_1:pnt_2+1] = list(reversed(creat[pnt_1:pnt_2+1]))
        if random.random() < self.mut_3_prob:
            engi = [number-1 for number in engi if number != 0]
            # engi = [number - 2 for number in engi if number > 1]
            while(sum(engi) != self.atms_number):
                engi[random.randint(0, len(engi)-1)] += 1
        return creat, engi

    def two_opt(self, creature):
        sum_dist = np.zeros(len(creature))
        for j in range(len(creature)):
            mat_path = np.zeros((self.dist.shape[0], self.dist.shape[1]))
            path = creature[j]
            if len(path) != 0:
                for v in range(len(path)):
                    if v == 0:
                        mat_path[self.engineers[j], path[v]] = 1
                    else:
                        mat_path[path[v - 1] + service_centers, path[v]] = 1
                mat_path = mat_path * dist
                sum_dist[j] = (np.sum(mat_path) + self.dist[self.engineers[j], path[-1]]) / velocity + repair_time * len(path)
        for u in range(len(creature)):
            best_path = creature[u].copy()
            while True:
                previous_best_path = best_path.copy()
                for x in range(len(creature[u])-1):
                    for y in range(x + 1, len(creature[u])):
                        path = best_path.copy()
                        if len(path) != 0:
                            path = path[:x] + list(reversed(path[x:y])) + path[y:]      # 2-opt swap
                            mat_path = np.zeros((self.dist.shape[0], self.dist.shape[1]))
                            for v in range(len(path)):
                                if v == 0:
                                    mat_path[self.engineers[u], path[v]] = 1
                                else:
                                    mat_path[path[v - 1] + self.service_centers, path[v]] = 1
                            mat_path = mat_path * self.dist
                            sum_dist_path = (np.sum(mat_path) + self.dist[self.engineers[u], path[-1]]) / self.velocity + self.repair_time * len(path)
                            if sum_dist_path < sum_dist[u]:
                                best_path = path.copy()
                                creature[u] = path.copy()
                if previous_best_path == best_path:
                    break
        return creature

    def crossover_mutation(self, population, birth_prob):
        new_population = []
        for i in range(round(len(population)/2)):
            prob = np.random.rand(birth_prob.size) - birth_prob
            pair = np.zeros(2).astype(int)
            pair[0] = np.argmin(prob)
            pair[1] = random.randint(0, prob.size-1)
            engi_1 = [len(population[pair[0]][v]) for v in range(len(population[pair[0]]))]
            engi_2 = [len(population[pair[1]][v]) for v in range(len(population[pair[1]]))]
            parent_1 = []
            parent_2 = []
            for j in range(len(engi_1)):
                parent_1 += population[pair[0]][j]
            for j in range(len(engi_2)):
                parent_2 += population[pair[1]][j]
            creat_1 = [-1] * len(parent_1)
            creat_2 = [-1] * len(parent_2)
            cross_point_1 = random.randint(0, len(parent_1) - 1)
            cross_point_2 = random.randint(0, len(parent_2) - 1)
            node_1 = parent_1[cross_point_1:]
            node_2 = parent_2[cross_point_2:]
            w = 0
            for v in range(len(creat_1)):
                if parent_2[v] not in node_1:
                    creat_1[v] = parent_2[v]
                else:
                    creat_1[v] = node_1[w]
                    w += 1
            w = 0
            for v in range(len(creat_2)):
                if parent_1[v] not in node_2:
                    creat_2[v] = parent_1[v]
                else:
                    creat_2[v] = node_2[w]
                    w += 1
            # mutations
            creat_1, engi_1 = self.mutate(creat_1, engi_1)
            creat_2, engi_2 = self.mutate(creat_2, engi_2)
            # children
            child_1 = []
            engi_sum = 0
            for v in range(len(engi_1)):
                child_1.append(creat_1[engi_sum:engi_sum+engi_1[v]])
                engi_sum += engi_1[v]
            child_2 = []
            engi_sum = 0
            for v in range(len(engi_2)):
                child_2.append(creat_2[engi_sum:engi_sum + engi_2[v]])
                engi_sum += engi_2[v]
            together = [child_1, child_2, population[pair[0]], population[pair[1]]]
            fit = np.array([self.fitness(creature) for creature in together])
            fit = fit.argsort()
            if self.two_opt_search:
                new_population.append(self.two_opt(together[fit[0]]))
                new_population.append(self.two_opt(together[fit[1]]))
            else:
                new_population.append(together[fit[0]])
                new_population.append(together[fit[1]])
        return new_population

    def plot_paths(self, paths, ax=None):
        # plt.clf()
        # plt.title('Best path overall')
        if ax!=None:
            # ax.cla()
            for v in range(self.service_centers):
                ax.scatter(self.points_locations[v, 0], self.points_locations[v, 1], c='r')
            for v in range(self.atms_number):
                ax.scatter(self.points_locations[v+self.service_centers, 0], self.points_locations[v+self.service_centers, 1], c='b')
            for v in range(len(paths)):
                if len(paths[v]) != 0:
                    path_locations = self.points_locations[self.service_centers:]
                    path_locations = path_locations[np.array(paths[v])]
                    path_locations = np.vstack((self.points_locations[self.engineers[v]], path_locations))
                    path_locations = np.vstack((path_locations, self.points_locations[self.engineers[v]]))
                    ax.plot(path_locations[:, 0], path_locations[:, 1])
        else:
            for v in range(self.service_centers):
                plt.scatter(self.points_locations[v, 0], self.points_locations[v, 1], c='r')
            for v in range(self.atms_number):
                plt.scatter(self.points_locations[v+self.service_centers, 0], self.points_locations[v+self.service_centers, 1], c='b')
            for v in range(len(paths)):
                if len(paths[v]) != 0:
                    path_locations = self.points_locations[self.service_centers:]
                    path_locations = path_locations[np.array(paths[v])]
                    path_locations = np.vstack((self.points_locations[self.engineers[v]], path_locations))
                    path_locations = np.vstack((path_locations, self.points_locations[self.engineers[v]]))
                    plt.plot(path_locations[:, 0], path_locations[:, 1])

            # plt.show()
            # plt.pause(0.0001)


    def get_path(self):
        np.random.seed(2)
        random.seed(1)
        # self.engineers = []
        num_agent_center=1
        for i in range(self.service_centers):
            for j in range(num_agent_center):
                self.engineers.append(i)
        self.engineers = np.array(self.engineers)
        print('Engineers: {}'.format(self.engineers))
        self.dist = np.zeros((self.atms_number+self.service_centers, self.atms_number))

        # points_locations = np.random.randint(-50, 50, (self.service_centers+self.atms_number)*2)
        # print("points_location", points_locations)
        # self.points_locations = self.points_locations.reshape((self.service_centers+self.atms_number, 2))
        # print("service_centers", self.service_centers)
        # print("points_location", self.points_locations)
        # input("--")

        for i in range(self.dist.shape[0]):
            for j in range(self.dist.shape[1]):
                self.dist[i, j] = math.sqrt((self.points_locations[i, 0] - self.points_locations[j + self.service_centers, 0]) ** 2 +
                                       (self.points_locations[i, 1] - self.points_locations[j + self.service_centers, 1]) ** 2)
                if j+self.service_centers == i:
                    self.dist[i][j] = 0
        # random population creation
        population = []
        for i in range(self.population_size):
            atms_range = list(range(self.atms_number))
            pop = [0] * self.engineers.size
            for j in range(self.engineers.size):
                pop[j] = []
                if len(atms_range) != 0:
                    if j != self.engineers.size-1:
                        for v in range(random.randint(1, round(2*self.atms_number/self.engineers.size))):
                            pop[j].append(random.choice(atms_range))
                            atms_range.remove(pop[j][-1])
                            if len(atms_range) == 0:
                                break
                    else:
                        for v in range(len(atms_range)):
                            pop[j].append(random.choice(atms_range))
                            atms_range.remove(pop[j][-1])
            population.append(pop)
        fitness_result = self.fitness_pop(population)
        best_mean_creature_result = np.mean(fitness_result)
        best_creature_result = np.min(fitness_result)

        best_selection_prob = self.birth_prob(fitness_result)
        selection_prob = best_selection_prob
        new_population = population.copy()
        best_path=None
        # plot_paths(population[np.argmin(fitness_result)])
        for i in range(self.generations):
            new_population = self.crossover_mutation(population, selection_prob)
            fitness_result = self.fitness_pop(new_population)
            mean_creature_result = np.mean(fitness_result)
            if np.min(fitness_result) < best_creature_result:
                # self.plot_paths(population[np.argmin(fitness_result)])
                best_path=population[np.argmin(fitness_result)]
                best_creature_result = np.min(fitness_result)
            if mean_creature_result < best_mean_creature_result:
                best_mean_creature_result = mean_creature_result
                best_selection_prob = self.birth_prob(fitness_result)
                selection_prob = best_selection_prob
                population = new_population.copy()
            # print('Mean population time: {0} Best time: {1}'.format(best_mean_creature_result, best_creature_result))
        # if self.service_centers==1
            # best_path=best_path[0]
        if best_path!=None:
            return best_path
        else:
            print("population[best_creature_result]", population[0])
            return population[0]

        # self.plot_paths(best_path)
        # print("fitness_result", fitness_result)





'''
# Bank parameters
atms_number = 20         # ATM quantity
self.service_centers = 2     # service centers quantity
velocity = 100             # 100 / hour
repair_time = 0         # 0.5 hour
max_engi = 3              # maximum number of engineers in one service center

# genetic parameters
population_size = 50    # population size (even number!)
generations = 1000       # population's generations
mut_1_prob = 0.4         # prob of replacing together two atms in combined path
mut_2_prob = 0.6      # prob of reversing the sublist in combined path
mut_3_prob = 0.8     # probability of changing the length of paths for engineers
two_opt_search = False  # better convergence, lower speed for large quantity of atms

# seed
np.random.seed(2)
random.seed(1)
plt.ion()
engineers = []
num_agent=1
for i in range(service_centers):
    # for j in range(random.randint(1, max_engi)):
    for j in range(num_agent):
        engineers.append(i)
engineers = np.array(engineers)
print('Engineers: {}'.format(engineers))
# input("--")
dist = np.zeros((atms_number+service_centers, atms_number))
self.points_locations = np.random.randint(-50, 50, (service_centers+atms_number)*2)
points_locations = points_locations.reshape((service_centers+atms_number, 2))
for i in range(dist.shape[0]):
    for j in range(dist.shape[1]):
        dist[i, j] = math.sqrt((points_locations[i, 0] - points_locations[j + service_centers, 0]) ** 2 +
                               (points_locations[i, 1] - points_locations[j + service_centers, 1]) ** 2)
        if j+service_centers == i:
            dist[i][j] = 0
# random population creation
population = []
for i in range(population_size):
    atms_range = list(range(atms_number))
    pop = [0] * engineers.size
    for j in range(engineers.size):
        pop[j] = []
        if len(atms_range) != 0:
            if j != engineers.size-1:
                for v in range(random.randint(1, round(2*atms_number/engineers.size))):
                    pop[j].append(random.choice(atms_range))
                    atms_range.remove(pop[j][-1])
                    if len(atms_range) == 0:
                        break
            else:
                for v in range(len(atms_range)):
                    pop[j].append(random.choice(atms_range))
                    atms_range.remove(pop[j][-1])
    population.append(pop)
fitness_result = fitness_pop(population)
best_mean_creature_result = np.mean(fitness_result)
best_creature_result = np.min(fitness_result)
best_selection_prob = birth_prob(fitness_result)
selection_prob = best_selection_prob
new_population = population.copy()
plot_paths(population[np.argmin(fitness_result)])
for i in range(generations):
    new_population = crossover_mutation(population, selection_prob)
    fitness_result = fitness_pop(new_population)
    mean_creature_result = np.mean(fitness_result)
    if np.min(fitness_result) < best_creature_result:
        plot_paths(population[np.argmin(fitness_result)])
        best_creature_result = np.min(fitness_result)
    if mean_creature_result < best_mean_creature_result:
        best_mean_creature_result = mean_creature_result
        best_selection_prob = birth_prob(fitness_result)
        selection_prob = best_selection_prob
        population = new_population.copy()
    print('Mean population time: {0} Best time: {1}'.format(best_mean_creature_result, best_creature_result))
plt.ioff()
plt.show()
'''
