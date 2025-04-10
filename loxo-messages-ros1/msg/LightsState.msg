# Message for light state of the vehicle
Header header

BoolCommanded isLeftTurningLight    # Left turning indicator | Clignotant gauche | Signofile gauche | Clinoteur gauche
BoolCommanded isRightTurningLight   # Right turning indicator | Clignotant droit | Signofile droit | Clinoteur droit
BoolCommanded isHazardLight         # Caution / Warning / Hazard lights (all indicators) | Feux de détresse / d'avertissement / de danger (tous les clignotants, signofiles, clinoteurs)

BoolCommanded isSidelight           # Front lights that are always on | Feux de position avant
BoolCommanded isLowBeamLight        # Low beam light | Feux de croisement
BoolCommanded isHighBeamLight       # High beam light | Feux de route
BoolCommanded isFrontFoglight       # Front fog light | Feux antibrouillard avant

BoolCommanded isTaillight           # Taillight | Feux arrière
BoolCommanded isRearLight           # Backup light | Lumière de recule
BoolCommanded isBrakeLight          # Brake light | Feux de freinage
BoolCommanded isRearFoglight        # Rear fog light | Feux antibrouillard arrière

string extraInfo