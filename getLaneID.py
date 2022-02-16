import numpy as np

def getLaneID(x,y,cameraid=2,verbose=0):

    # Define the bounding points for the vehicle lanes
    if cameraid == 1:
        slipNS_coeffs = [3.8480816e-10, -1.8245187e-06, 0.0031493688, -3.5215877, 2984.3939]
        slipOS_coeffs = [-1.7715407e-10, 9.7503725e-07, -0.0019015813, 0.7328096, 1201.8836]
        slowNS_coeffs = [1.1561612e-10, -4.0644495e-07, 0.0004869708, -0.89861606, 1326.6585]
        medNS_coeffs = [5.4930498e-11, -1.6575714e-07, 0.00016463682, -0.60160233, 1058.7758]
        fastNS_coeffs = [5.772378e-11, -1.791067e-07, 0.0001814117, -0.5133629, 889.1925]
        fastOS_coeffs = [3.757816e-11, -1.092216e-07, 0.0001100817, -0.4311191, 771.0408]
    elif cameraid == 2:
        slipNS_coeffs = [2.532593e-10, -8.683776e-07, 0.001053468, 0.4899414, -111.0162]
        slipOS_coeffs = [5.309146e-11, -2.335239e-07, 0.0003818898, 0.4781764, -136.4309]
        slowNS_coeffs = [3.530467e-12, -2.203655e-08, 5.176177e-05, 0.5388626, -136.525]
        medNS_coeffs = [-1.544132e-11, 5.70252e-08, -5.302592e-05, 0.4934357, -148.7334]
        fastNS_coeffs = [1.939304e-11, -8.848078e-08, 0.0001549821, 0.3017643, -129.4931]
        fastOS_coeffs = [-9.484667e-12, 3.511012e-08, -2.498244e-05, 0.3478557, -149.9981]
    elif cameraid == 3:
        slipNS_coeffs = [1.726913e-09, -8.4631153e-06, 0.015424324, -13.732914, 5965.4385]
        slipOS_coeffs = [2.0663446e-10, -7.6479374e-07, 0.0010504168, -1.5658187, 1676.2516]
        slowNS_coeffs = [2.399259e-10, -8.8480836e-07, 0.0011659049, -1.5527044, 1582.8937]
        medNS_coeffs = [6.0753514e-11, -1.7515989e-07, 0.00017287308, -0.76880642, 1118.0835]
        fastNS_coeffs = [3.186552e-11, -7.617913e-08, 6.415323e-05, -0.6015218, 899.8654]
        fastOS_coeffs = [2.815241e-11, -7.268973e-08, 7.517764e-05, -0.5226031, 748.1428]
    elif cameraid == 4:
        slipNS_coeffs = [3.72188e-10, -1.17827e-06, 0.00135098, 0.369299, -58.1023]
        slipOS_coeffs = [8.5967e-11, -3.99321e-07, 0.000668694, 0.292707, -63.2118]
        slowNS_coeffs = [8.95336e-11, -3.87699e-07, 0.000596693, 0.334393, -72.0982]
        medNS_coeffs = [3.01169e-11, -1.50319e-07, 0.000276151, 0.35916, -91.6472]
        fastNS_coeffs = [2.64961e-11, -1.42535e-07, 0.000280705, 0.24848, -84.6373]
        fastOS_coeffs = [2.05336e-11, -1.12953e-07, 0.000231406, 0.20902, -90.4761]
    elif cameraid == 5:
        slipNS_coeffs = [1.0054154e-09, -5.2192341e-06, 0.010064861, -9.8402549, 5057.0127]
        slipOS_coeffs = [1.1146685e-10, -4.1446313e-07, 0.00056439112, -1.1582247, 1535.6429]
        slowNS_coeffs = [5.7612227e-11, -1.9695928e-07, 0.00024274092, -0.9431503, 1467.3901]
        medNS_coeffs = [4.2051677e-11, -1.2866629e-07, 0.00013695768, -0.68796986, 1100.5433]
        fastNS_coeffs = [1.044752e-11, -2.121854e-08, 2.327063e-05, -0.5210934, 856.5486]
        fastOS_coeffs = [-1.00515e-12, 1.577116e-08, -1.424127e-05, -0.4235832, 696.6871]
    elif cameraid == 6:
        slipNS_coeffs = [5.4894e-10, -1.7412e-06, 0.0019485, 0.13286, -7.8031]
        slowNS_coeffs = [6.08446e-11, -2.91506e-07, 0.00051183, 0.301933, -20.0234]
        medNS_coeffs = [5.80484e-11, -2.78741e-07, 0.000494916, 0.153959, -19.1423]
        fastNS_coeffs = [2.95579e-11, -1.50699e-07, 0.000304921, 0.153328, -32.8859]
        fastOS_coeffs = [1.11982e-12, -2.92984e-08, 0.000126114, 0.187191, -52.8338]
    elif cameraid == 7:
        slipNS_coeffs = [1.265023e-09, -6.8085504e-06, 0.013531457, -12.92509, 5964.2248]
        slowNS_coeffs = [1.2699223e-10, -4.9341267e-07, 0.0007041839, -1.256482, 1585.3535]
        medNS_coeffs = [2.2713741e-11, -4.0388627e-08, 1.4736669e-05, -0.62696177, 1114.8988]
        fastNS_coeffs = [2.672686e-12, 1.264511e-08, -1.334083e-05, -0.5093305, 875.4539]
        fastOS_coeffs = [2.760263e-12, 3.529875e-09, 7.718145e-06, -0.4320097, 708.8859]
    elif cameraid == 8:
        slipNS_coeffs = [2.21544e-10, -8.32432e-07, 0.00112647, 0.278224, 28.4732]
        slowNS_coeffs = [3.87131e-11, -1.7754e-07, 0.000310267, 0.40258, 17.4473]
        medNS_coeffs = [2.3389e-11, -1.2875e-07, 0.00027346, 0.24826, 5.5543]
        fastNS_coeffs = [2.9251e-11, -1.3779e-07, 0.00025532, 0.17965, -9.5755]
        fastOS_coeffs = [1.16384e-11, -7.23949e-08, 0.000175843, 0.144437, -24.2946]
    elif cameraid == 9:
        slipNS_coeffs = [2.2957052e-10, -7.0964611e-07, 0.001011917, -2.1108036, 2281.9915]
        slowNS_coeffs = [-2.8146645e-11, 8.8830357e-07, -0.0023310432, 0.75608248, 1373.2965]
        medNS_coeffs = [8.8242507e-11, -9.3169436e-08, -4.249189e-05, -0.83725118, 1312.7396]
        fastNS_coeffs = [8.188058e-11, -1.582859e-07, 0.0001245458, -0.7145562, 989.4404]
        fastOS_coeffs = [2.71563e-11, -2.027336e-08, 1.885569e-05, -0.5393854, 764.716]
    elif cameraid == 10:
        slowNS_coeffs = [5.285306e-10, -2.128071e-06, 0.003248771, -1.046702, 139.9227]
        medNS_coeffs = [1.07079e-10, -5.95253e-07, 0.00120153, -0.239106, 29.1563]
        fastNS_coeffs = [8.07031e-11, -4.32349e-07, 0.000876155, -0.165416, 19.4946]
        fastOS_coeffs = [3.32519e-11, -1.96856e-07, 0.000459824, 0.0166765, -16.0989]
        
    if cameraid < 10:
        slipNS_y = x**4*slipNS_coeffs[0] + x**3*slipNS_coeffs[1] + x**2*slipNS_coeffs[2] + x*slipNS_coeffs[3] + slipNS_coeffs[4]
    if cameraid < 6:
        slipOS_y = x**4*slipOS_coeffs[0] + x**3*slipOS_coeffs[1] + x**2*slipOS_coeffs[2] + x*slipOS_coeffs[3] + slipOS_coeffs[4]

    slowNS_y = x**4*slowNS_coeffs[0] + x**3*slowNS_coeffs[1] + x**2*slowNS_coeffs[2] + x*slowNS_coeffs[3] + slowNS_coeffs[4]
    medNS_y = x**4*medNS_coeffs[0] + x**3*medNS_coeffs[1] + x**2*medNS_coeffs[2] + x*medNS_coeffs[3] + medNS_coeffs[4]
    fastNS_y = x**4*fastNS_coeffs[0] + x**3*fastNS_coeffs[1] + x**2*fastNS_coeffs[2] + x*fastNS_coeffs[3] + fastNS_coeffs[4]
    fastOS_y = x**4*fastOS_coeffs[0] + x**3*fastOS_coeffs[1] + x**2*fastOS_coeffs[2] + x*fastOS_coeffs[3] + fastOS_coeffs[4]

    if cameraid < 6:
        if (y <= slipNS_y) and (y > slipOS_y):
            # Slip = 0
            if verbose:
                print(f"Matched lane 0 for co-ords: {x} {y}")
            return 0
    elif cameraid < 10:
        if (y <= slipNS_y) and (y > slowNS_y):
            # Slip = 0
            if verbose:
                print(f"Matched lane 0 for co-ords: {x} {y}")
            return 0
    
    if (y <= slowNS_y) and (y > medNS_y):
        # Slow lane = 1
        if verbose:
            print(f"Matched lane 1 for co-ords: {x} {y}")    
        return 1
    elif (y <= medNS_y) and (y > fastNS_y):      
        if verbose:
            print(f"Matched lane 2 for co-ords: {x} {y}")
        # Middle lane = 2
        return 2
    elif (y <= fastNS_y) and (y > fastOS_y):
        if verbose:
            print(f"Matched lane 3 for co-ords: {x} {y}")
        # Fast lane = 3
        return 3
    else:
        if verbose:
            print(f"[!] Object out of bounds: failed to find lane for co-ords: {x} {y}")
        # print(f"slipNS_coeffs: {slipNS_coeffs}, targetY: {x*slipNS_coeffs[0] + slipNS_coeffs[1]}")
        # print(f"slipOS_coeffs: {slipOS_coeffs}, targetY: {x*slipOS_coeffs[0] + slipOS_coeffs[1]}")
        # print(f"slowNS_coeffs: {slowNS_coeffs}, targetY: {x*slowNS_coeffs[0] + slowNS_coeffs[1]}")
        # print(f"medNS_coeffs: {medNS_coeffs}, targetY: {x*medNS_coeffs[0] + medNS_coeffs[1]}")
        # print(f"fastNS_coeffs: {fastNS_coeffs}, targetY: {x*fastNS_coeffs[0] + fastNS_coeffs[1]}")
        # Outside bounds = 4
        return 4