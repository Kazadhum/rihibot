import math


def main() -> None:
    pos_vectors = [
        [
            -7.63,
            -82.05,
            119.75,
            -213.55,
            -80.58,
            -3.08,
        ],
        [
            1.57,
            -47.02,
            65.32,
            -187.07,
            -105.97,
            -3.07,
        ],
        [
            -19.54,
            -43.42,
            64.42,
            -198.85,
            -63.69,
            -3.07,
        ],
        [
            2.00,
            8.35,
            -46.93,
            -142.22,
            -104.33,
            -3.07,
        ],
        [
            -4.45,
            -46.29,
            31.31,
            -137.02,
            -87.69,
            -65.08,
        ],
        [
            -48.15,
            -22.34,
            32.74,
            -183.30,
            -10.47,
            -11.47,
        ],
        [
            14.49,
            -51.85,
            98.24,
            -225.61,
            -120.26,
            44.36,
        ],
        [
            8.16,
            -75.16,
            92.63,
            -187.85,
            -108.30,
            -95.99,
        ],
        [
            -9.26,
            -75.22,
            146.55,
            -252.71,
            -93.95,
            -7.29,
        ],
    ]

    rad_pos_vectors= []

    for pos_vector in pos_vectors:
        new_pos_vector = []
        for joint_pos in pos_vector:
            new_pos_vector.append((joint_pos/180)*math.pi)
        rad_pos_vectors.append(new_pos_vector)    

    for vec in rad_pos_vectors:
        print(f"{vec}")


if __name__ == "__main__":
    main()
