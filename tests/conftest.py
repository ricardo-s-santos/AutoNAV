"""Contains the fixtures needed for the tests."""

import os

import pytest
from numpy import array, insert

from autonav.file_handlers import readpathfile


@pytest.fixture(scope="session")
def default_values():
    """Defines the default values to be used in the GTRS and WLS tests."""
    root_dir = os.path.dirname(os.path.realpath(__file__))
    filename = root_dir + "/path_files/Path_small.csv"
    destinations = readpathfile(filename)
    n = 8  # Number of anchors
    b = 200  # Area border in meters
    k = 50  # Number of measurement samples
    a_i = array(
        [
            [0, 0, 0],
            [0, b, 0],
            [b / 2, 0, 0],
            [b / 2, b, 0],
            [0, 0, b / 8],
            [0, b, b / 8],
            [b / 2, 0, b / 8],
            [b / 2, b, b / 8],
        ]
    ).T
    initial_uav_position = [10, 10, 5]
    v_max = 2
    tau = 4
    gamma = 2
    return [a_i, n, k, destinations, initial_uav_position, v_max, tau, gamma]


@pytest.fixture(scope="session")
def ideal_trajectory():
    """Contains the ideal trajectory for the plots.py tests."""
    root_dir = os.path.dirname(os.path.realpath(__file__))
    filename = root_dir + "/path_files/Path_small.csv"
    ideal_trajectory = readpathfile(filename)
    ideal_trajectory = insert(ideal_trajectory, 0, [10, 10, 5], axis=0)
    return ideal_trajectory


@pytest.fixture(scope="session", params=[5, 24612, 93456558])
def seeds(request):
    """Contains the seeds for the GTRS and WLS tests."""
    return request.param


@pytest.fixture(scope="session")
def metrics_trajectories():
    """Contains the trajectories for the metrics.py tests."""
    estimated_trajectory = array(
        [
            [9.99999962, 9.99999996, 4.99999754],
            [11.99410947, 9.84660779, 5.00000705],
            [13.98821739, 9.69321436, 5.00000066],
            [15.98232633, 9.53982188, 5.0000055],
            [17.97643436, 9.38642838, 4.99999871],
            [19.97054325, 9.23303628, 5.00000696],
            [21.96465149, 9.07964328, 5.00000546],
            [23.95875955, 8.92624995, 5.00000067],
            [25.95286755, 8.77285661, 4.9999958],
            [27.94697505, 8.61946163, 4.99997503],
            [29.94108338, 8.46607013, 4.99998826],
        ]
    )
    true_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.99410897, 9.846607, 5.00000008],
            [13.98821794, 9.69321398, 4.99999985],
            [15.98232691, 9.53982097, 4.99999983],
            [17.97643588, 9.38642795, 4.99999964],
            [19.97054485, 9.23303494, 4.99999969],
            [21.96465382, 9.0796419, 4.99999944],
            [23.95876279, 8.92624886, 4.99999923],
            [25.95287175, 8.77285584, 4.99999921],
            [27.94698072, 8.61946283, 4.99999938],
            [29.9410897, 8.46606991, 5.00000044],
        ]
    )
    return [estimated_trajectory, true_trajectory]


@pytest.fixture(scope="session")
def expected_trajectories_gtrs_sigma_0():
    """Defines the expected GTRS trajectories for sigma = 0."""
    expected_estimated_trajectory = array(
        [
            [9.99999962, 9.99999996, 4.99999754],
            [11.96116071, 9.60776772, 4.99999721],
            [13.92232185, 9.21553558, 4.99999791],
            [15.88348199, 8.82330229, 4.999986],
            [17.84464418, 8.4310719, 5.00000478],
            [19.80580439, 8.03883851, 4.99999227],
            [21.7669659, 7.64660808, 5.00000907],
            [23.72812616, 7.25437476, 4.99999755],
            [25.68928652, 6.8621417, 4.99998893],
            [27.65044726, 6.4699106, 4.99999824],
            [29.61160763, 6.07767839, 4.99999728],
            [31.57276764, 5.68544437, 4.99998068],
            [33.07008382, 5.38598328, 4.99999949],
            [33.54487575, 5.29102508, 5.00000119],
            [33.81479041, 5.23704155, 4.99999637],
            [33.99385789, 5.20122822, 4.99999784],
            [34.12290389, 5.17541767, 4.99998662],
            [34.24102725, 7.17192767, 5.00000128],
            [34.35915046, 9.16843643, 5.00000358],
            [34.47727341, 11.16494388, 4.99999259],
            [34.59539692, 13.16145365, 5.00000795],
            [34.71352035, 15.15796275, 5.0000136],
            [34.83164322, 17.15447008, 4.99999528],
            [34.89163075, 18.16836999, 4.99999724],
            [34.91648546, 18.58846157, 4.99999418],
            [34.93124667, 18.83795218, 4.9999954],
            [34.9412509, 19.00704167, 4.9999953],
            [34.97057627, 21.0002213, 4.99999274],
            [34.97793246, 21.50016542, 5.00000492],
            [34.98206936, 21.78138256, 4.99998618],
            [34.98480106, 21.96703143, 4.99999182],
            [34.98676396, 22.10042423, 4.99999836],
            [33.41816758, 23.34119101, 4.99999199],
            [31.84957187, 24.58195877, 5.00000204],
            [30.28097588, 25.82272609, 5.00000695],
            [28.71237982, 27.0634932, 5.00001071],
            [27.14378235, 28.30425925, 4.9999971],
            [26.41131343, 28.88364661, 5.00000842],
            [26.09386208, 29.13474974, 4.99998995],
            [25.90316202, 29.28559606, 5.00000505],
        ]
    )
    expected_true_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.96116135, 9.60776774, 5.00000019],
            [13.92232271, 9.21553548, 5.00000043],
            [15.88348406, 8.82330321, 5.00000062],
            [17.84464544, 8.43107107, 5.00000206],
            [19.80580677, 8.03883872, 5.00000151],
            [21.76696814, 7.64660653, 5.00000251],
            [23.72812945, 7.25437408, 5.00000117],
            [25.68929081, 6.86214181, 5.00000159],
            [27.6504522, 6.46990974, 5.00000393],
            [29.61161355, 6.07767746, 5.0000044],
            [31.5727749, 5.68544522, 5.00000538],
            [33.07009182, 5.38598275, 5.00001383],
            [33.54488402, 5.2910243, 5.00001395],
            [33.81479892, 5.23704128, 5.00001373],
            [33.9938665, 5.20122782, 5.00001428],
            [34.12291273, 5.1754186, 5.00001456],
            [34.24103613, 7.17192727, 5.00001636],
            [34.35915954, 9.16843593, 5.00001616],
            [34.47728299, 11.1649446, 5.0000155],
            [34.59540654, 13.16145326, 5.00001717],
            [34.71353011, 15.15796192, 5.00001485],
            [34.83165376, 17.15447057, 5.00000925],
            [34.89164151, 18.16837059, 5.00001093],
            [34.91649645, 18.5884625, 5.00001156],
            [34.93125771, 18.83795313, 5.00001259],
            [34.94126201, 19.0070427, 5.00001326],
            [34.97058802, 21.00022293, 5.00001561],
            [34.97794394, 21.50016639, 5.00001742],
            [34.98208159, 21.7813848, 5.0000165],
            [34.98481321, 21.96703345, 5.0000186],
            [34.98677593, 22.1004259, 5.00001966],
            [33.41817897, 23.34119323, 5.00001992],
            [31.84958202, 24.58196058, 5.00002141],
            [30.28098492, 25.82272773, 5.00002094],
            [28.71238766, 27.0634947, 5.00001888],
            [27.14379018, 28.30426137, 5.00001435],
            [26.41132002, 28.88364832, 5.00001534],
            [26.09387005, 29.13475225, 5.00001345],
            [25.90316866, 29.28559798, 5.0000152],
        ]
    )
    return [
        expected_estimated_trajectory,
        expected_true_trajectory,
    ]


@pytest.fixture(scope="session")
def expected_trajectories_wls_sigma_0():
    """Defines the expected WLS trajectories for sigma = 0."""
    expected_estimated_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.961, 9.608, 5.0],
            [13.922, 9.216, 5.0],
            [15.883, 8.824, 5.0],
            [17.844, 8.432, 5.0],
            [19.805, 8.04, 5.0],
            [21.766, 7.648, 5.0],
            [23.727, 7.256, 5.0],
            [25.688, 6.864, 5.0],
            [27.649, 6.471, 5.0],
            [29.61, 6.079, 5.0],
            [31.571, 5.686, 5.0],
            [33.07, 5.386, 5.0],
            [33.545, 5.291, 5.0],
            [33.815, 5.237, 5.0],
            [33.994, 5.201, 5.0],
            [34.123, 5.175, 5.0],
            [34.241, 7.172, 5.0],
            [34.359, 9.169, 5.0],
            [34.477, 11.166, 5.0],
            [34.595, 13.163, 5.0],
            [34.713, 15.16, 5.0],
            [34.831, 17.156, 5.0],
            [34.891, 18.169, 5.0],
            [34.916, 18.589, 5.0],
            [34.931, 18.838, 5.0],
            [34.941, 19.007, 5.0],
            [34.97, 21.0, 5.0],
            [34.978, 21.5, 5.0],
            [34.982, 21.781, 5.0],
            [34.985, 21.967, 5.0],
            [34.987, 22.1, 5.0],
            [33.418, 23.341, 5.0],
            [31.849, 24.582, 5.0],
            [30.28, 25.823, 5.0],
            [28.711, 27.064, 5.0],
            [27.143, 28.305, 5.0],
            [26.411, 28.884, 5.0],
            [26.094, 29.135, 5.0],
            [25.903, 29.286, 5.0],
        ]
    )
    expected_true_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.961, 9.608, 5.0],
            [13.922, 9.216, 5.0],
            [15.883, 8.824, 5.0],
            [17.844, 8.432, 5.0],
            [19.805, 8.04, 5.0],
            [21.766, 7.648, 5.0],
            [23.727, 7.256, 5.0],
            [25.688, 6.864, 5.0],
            [27.649, 6.471, 5.0],
            [29.61, 6.079, 5.0],
            [31.571, 5.686, 5.0],
            [33.07, 5.386, 5.0],
            [33.545, 5.291, 5.0],
            [33.815, 5.237, 5.0],
            [33.994, 5.201, 5.0],
            [34.123, 5.175, 5.0],
            [34.241, 7.172, 5.0],
            [34.359, 9.169, 5.0],
            [34.477, 11.166, 5.0],
            [34.595, 13.163, 5.0],
            [34.713, 15.16, 5.0],
            [34.831, 17.156, 5.0],
            [34.891, 18.169, 5.0],
            [34.916, 18.589, 5.0],
            [34.931, 18.838, 5.0],
            [34.941, 19.007, 5.0],
            [34.97, 21.0, 5.0],
            [34.978, 21.5, 5.0],
            [34.982, 21.781, 5.0],
            [34.985, 21.967, 5.0],
            [34.987, 22.1, 5.0],
            [33.418, 23.341, 5.0],
            [31.849, 24.582, 5.0],
            [30.28, 25.823, 5.0],
            [28.711, 27.064, 5.0],
            [27.143, 28.305, 5.0],
            [26.411, 28.884, 5.0],
            [26.094, 29.135, 5.0],
            [25.903, 29.286, 5.0],
        ]
    )
    return [
        expected_estimated_trajectory,
        expected_true_trajectory,
    ]
