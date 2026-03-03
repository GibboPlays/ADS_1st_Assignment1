def create_3d_car(x_pos, y_pos, points_list):
    """Crea un volume 3D denso (box pieno) per distinguere le forme"""
    # X: da x_pos a x_pos + 4m (lunghezza)
    # Y: da y_pos - 1m a y_pos + 1m (larghezza)
    # Z: da 0.2m a 1.5m (altezza)
    for x in range(int(x_pos*10), int((x_pos+4)*10), 1): # passo 10cm
        for y in range(int((y_pos-1)*10), int((y_pos+1)*10), 1):
            for z in range(2, 16, 1): # partiamo da 0.2m per non toccare il suolo (Z=0)
                points_list.append((x/10.0, y/10.0, z/10.0))

def genera_autostrada_densa():
    punti = []
    
    # 1. STRADA MOLTO DENSA (Passo 10cm)
    for x in range(0, 500, 1): # 50 metri
        for y in range(-60, 60, 2): # 12 metri di larghezza
            punti.append((x/10.0, y/10.0, 0.0))

    # 2. LE 4 AUTO (Distanze: 4m, 8m, 15m, 35m)
    create_3d_car(4.0, 0.0, punti)   # Pericolo (Corsia Centro)
    create_3d_car(8.0, -4.0, punti)  # Sicura (Corsia Destra)
    create_3d_car(15.0, 4.0, punti)  # Sicura (Corsia Sinistra)
    create_3d_car(35.0, 0.0, punti)  # Sicura (Lontana)

    with open("autostrada_test.pcd", "w") as f:
        f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(punti)}\nHEIGHT 1\nPOINTS {len(punti)}\nDATA ascii\n")
        for p in punti:
            f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
    
    print(f"Creato 'autostrada_test.pcd' con {len(punti)} punti.")

if __name__ == "__main__":
    genera_autostrada_densa()