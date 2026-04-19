def ema(prev, new, a):
    return a * new + (1 - a) * prev