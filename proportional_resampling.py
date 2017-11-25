import random


def proportional_resampling(particles, weights, num_samples):
    """
    This function takes in a list and samples with replacement
    in proportion to the weight objects weights. This is useful for
    particle filtering
    :param particles: list of objects to sample from
    :param weights: list of weights for the particles
    :param num_samples: number of objects to resample
    :return: a list containing sampled objects with length = num_samples
    """

    assert len(particles) == len(weights)
    p3 = []

    # draw index from random uniform
    index = int(random.random() * num_samples)
    beta = 0.0
    mw = max(weights)
    for i in range(num_samples):
        beta += random.random() * 2.0 * mw
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % num_samples
        p3.append(particles[index])
    return p3
