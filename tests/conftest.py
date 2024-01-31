"""This file contains the fixtures needed for the tests."""

import os

import pytest
from autonav.file_handlers import readpathfile
from numpy import array, insert


@pytest.fixture(scope="session")
def default_values():
    """This fixture defines the default values to be used in the algorithm tests."""
    ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
    filename = ROOT_DIR + "/path_files/Path_small.txt"
    destinations = readpathfile(filename)
    N = 8  # Number of anchors
    B = 200  # Area border in meters
    K = 50  # Number of measurement samples
    a_i = array(
        [
            [0, 0, 0],
            [0, B, 0],
            [B / 2, 0, 0],
            [B / 2, B, 0],
            [0, 0, B / 8],
            [0, B, B / 8],
            [B / 2, 0, B / 8],
            [B / 2, B, B / 8],
        ]
    ).T
    initial_uav_position = [10, 10, 5]
    return [a_i, N, K, destinations, initial_uav_position]


@pytest.fixture(scope="session")
def ideal_trajectory():
    """This fixture contains the ideal trajectory for the plots.py tests."""
    ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
    filename = ROOT_DIR + "/path_files/Path_small.txt"
    ideal_trajectory = readpathfile(filename)
    ideal_trajectory = insert(ideal_trajectory, 0, [10, 10, 5], axis=0)
    return ideal_trajectory


@pytest.fixture(scope="session", params=[5, 10, 20])
def seeds(request):
    """This fixture contains the seeds for the GTRS and WLS tests."""
    return request.param


@pytest.fixture(scope="session")
def metrics_trajectories():
    """This fixture contains the trajectories for the metrics.py tests."""
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
    """This fixture defines the expected GTRS trajectories for sigma = 0."""
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
def expected_trajectories_gtrs_sigma_1():
    """This fixture defines the expected GTRS trajectories for sigma = 1."""
    expected_estimated_trajectory = array(
        [
            [10.00930112, 10.11974317, 5.03590142],
            [11.97427759, 9.5486463, 4.88177854],
            [13.74242476, 9.25787825, 4.8657601],
            [15.74617403, 8.92943158, 5.10165607],
            [17.93724489, 8.31422023, 5.12123819],
            [19.87344384, 8.08957262, 5.13785152],
            [21.84039295, 7.83703433, 4.81145407],
            [23.60353808, 7.04617318, 5.0219039],
            [25.56340237, 6.91695865, 4.39482172],
            [27.61794754, 6.3300552, 4.8827867],
            [29.67607724, 6.15634883, 4.94099597],
            [31.56651921, 5.56668802, 5.5329315],
            [33.03220414, 5.32138176, 5.23458169],
            [33.49007959, 5.41223583, 5.18051586],
            [33.96550861, 5.20964921, 5.49995868],
            [33.99928719, 5.30510068, 4.62005905],
            [34.14472383, 5.22449482, 4.65731353],
            [34.52814871, 7.33251027, 4.43731015],
            [34.52531579, 9.24079886, 5.21394689],
            [34.35420898, 11.12104198, 4.61242967],
            [34.64330255, 13.29394835, 4.92211525],
            [34.67963484, 15.2617934, 5.47839183],
            [34.7872703, 16.95854208, 4.89680512],
            [35.03895113, 18.40382052, 4.97803748],
            [34.86999202, 18.63847656, 4.32796553],
            [34.86297063, 19.00397162, 5.13788919],
            [34.99160005, 19.02051837, 5.41277957],
            [35.02852289, 21.06072186, 5.07896776],
            [34.79425787, 21.44564048, 4.59764087],
            [35.024751, 21.85638893, 4.81964348],
            [34.87749931, 21.91045237, 4.98639399],
            [35.01482569, 22.22644923, 5.50116526],
            [33.54330252, 23.41634033, 4.39744475],
            [32.05836167, 24.48062534, 4.727215],
            [30.18610851, 25.57766014, 4.99805555],
            [28.75232239, 26.98927392, 5.24075716],
            [27.22820489, 28.37966403, 4.84962141],
            [26.39780993, 29.02200476, 5.12084784],
            [26.02149672, 29.17135781, 4.65347983],
            [26.0373089, 29.26013789, 4.62281133],
        ]
    )
    expected_true_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.95930455, 9.59860522, 4.99718528],
            [13.92136138, 9.21100815, 5.00725911],
            [15.88237217, 8.81821899, 5.01964274],
            [17.84195249, 8.41829656, 5.00929657],
            [19.80521174, 8.03695876, 4.99534678],
            [21.76467776, 7.63674125, 4.97748975],
            [23.71956829, 7.21529314, 5.00549869],
            [25.68808752, 6.86185615, 5.00171521],
            [27.64419679, 6.46449041, 5.12716242],
            [29.61226336, 6.10989564, 5.15841167],
            [31.56657961, 5.68542079, 5.18007099],
            [33.07752705, 5.43604243, 4.94554762],
            [33.57135017, 5.35539091, 4.88667877],
            [33.86872113, 5.27420321, 4.85112712],
            [34.01974912, 5.24359599, 4.7781369],
            [34.15897883, 5.20114717, 4.83099829],
            [34.27452387, 7.19726992, 4.87729412],
            [34.34889699, 9.19391774, 4.96598521],
            [34.43703167, 11.19157996, 4.92626166],
            [34.58197663, 13.18442314, 5.01325012],
            [34.68820009, 15.18146561, 5.03644397],
            [34.82243901, 17.16686172, 4.83598895],
            [34.90355907, 18.32665868, 4.87534018],
            [34.8957844, 18.64525725, 4.87972392],
            [34.92054933, 18.90461085, 5.00773828],
            [34.93793188, 19.03095975, 4.99024667],
            [34.94213097, 21.02028213, 4.78390028],
            [34.93521027, 21.49082247, 4.7647398],
            [34.97684014, 21.80533169, 4.84615317],
            [34.9732574, 21.97087083, 4.87226],
            [34.99004761, 22.12020654, 4.87412487],
            [33.41137223, 23.34558116, 4.79512427],
            [31.82965656, 24.56448657, 4.90668196],
            [30.25488224, 25.79590115, 4.96754237],
            [28.73305422, 27.09360649, 4.96811296],
            [27.17506635, 28.3436789, 4.86814907],
            [26.40656784, 28.90252598, 4.92001402],
            [26.10774247, 29.11160289, 4.89417903],
            [25.93406082, 29.25249414, 4.95309669],
        ]
    )
    return [
        expected_estimated_trajectory,
        expected_true_trajectory,
    ]


@pytest.fixture(scope="session")
def expected_trajectories_wls_sigma_0():
    """This fixture defines the expected WLS trajectories for sigma = 0."""
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


@pytest.fixture(scope="session")
def expected_trajectories_wls_sigma_1():
    """This fixture defines the expected WLS trajectories for sigma = 1."""
    expected_estimated_trajectory = array(
        [
            [9.878, 9.929, 4.965],
            [12.167, 9.826, 5.061],
            [13.974, 9.404, 5.037],
            [15.815, 8.94, 4.754],
            [17.99, 8.357, 5.102],
            [19.916, 8.111, 4.88],
            [21.964, 7.923, 5.201],
            [23.8, 7.085, 5.316],
            [25.482, 6.842, 4.445],
            [27.652, 6.317, 5.033],
            [29.673, 6.144, 4.95],
            [31.462, 5.573, 5.099],
            [33.188, 5.287, 5.203],
            [33.319, 5.347, 5.322],
            [33.989, 5.158, 5.19],
            [34.04, 5.287, 4.875],
            [34.15, 7.254, 4.739],
            [34.44, 9.326, 4.791],
            [34.508, 11.326, 5.196],
            [34.572, 13.243, 4.966],
            [34.685, 15.26, 4.872],
            [34.863, 17.305, 5.354],
            [34.902, 18.088, 5.38],
            [34.902, 18.637, 4.885],
            [34.903, 18.836, 4.337],
            [34.827, 19.098, 4.79],
            [35.054, 20.996, 5.416],
            [35.044, 21.428, 5.278],
            [34.949, 21.751, 4.607],
            [35.0, 21.971, 4.956],
            [34.913, 22.026, 4.784],
            [33.543, 23.447, 5.799],
            [31.986, 24.718, 4.555],
            [30.368, 25.714, 4.928],
            [28.64, 26.891, 4.861],
            [27.234, 28.156, 4.962],
            [26.285, 28.934, 5.018],
            [25.997, 29.248, 4.647],
            [25.929, 29.523, 5.21],
            [25.886, 29.412, 5.1],
        ]
    )
    expected_true_trajectory = array(
        [
            [10.0, 10.0, 5.0],
            [11.963, 9.615, 5.003],
            [13.92, 9.201, 4.998],
            [15.878, 8.791, 4.995],
            [17.837, 8.389, 5.02],
            [19.799, 8.002, 5.008],
            [21.758, 7.598, 5.024],
            [23.709, 7.16, 4.994],
            [25.674, 6.794, 4.939],
            [27.634, 6.415, 5.053],
            [29.603, 6.062, 5.044],
            [31.558, 5.642, 5.062],
            [33.144, 5.385, 5.018],
            [33.562, 5.319, 4.971],
            [33.929, 5.243, 4.901],
            [34.061, 5.222, 4.876],
            [34.191, 7.218, 4.893],
            [34.324, 9.213, 4.934],
            [34.429, 11.21, 4.973],
            [34.542, 13.206, 4.928],
            [34.668, 15.202, 4.938],
            [34.801, 17.197, 4.992],
            [34.848, 18.114, 4.872],
            [34.872, 18.58, 4.779],
            [34.889, 18.814, 4.799],
            [34.905, 19.009, 4.91],
            [34.99, 20.917, 5.013],
            [34.976, 21.43, 4.907],
            [34.967, 21.744, 4.852],
            [34.975, 21.949, 4.916],
            [34.975, 22.081, 4.922],
            [33.417, 23.334, 4.956],
            [31.834, 24.548, 4.808],
            [30.241, 25.753, 4.909],
            [28.678, 27.001, 4.93],
            [27.158, 28.299, 4.988],
            [26.349, 28.967, 5.002],
            [26.081, 29.189, 4.998],
            [25.919, 29.311, 5.055],
            [25.795, 29.375, 5.027],
        ]
    )
    return [
        expected_estimated_trajectory,
        expected_true_trajectory,
    ]
