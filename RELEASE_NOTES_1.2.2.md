# KiCad Track Gloss 1.2.2

Etat : version intermediaire de test plugin, sans campagne complete de
non-regression.

- Une connexion unique produit desormais jusqu'a trois geometries de gloss
  distinctes et entierement convergees avant le DRC natif.
- Les variantes conservent alternativement les intersections de pistes, les
  contacts de pads, ou tous les terminaux electriques. Elles couvrent notamment
  la translation locale d'un segment avec reprise de ses segments adjacents.
- Le portefeuille DRC 3/3 est maintenant utilise aussi pour une connexion
  unique ; la version 1.2.1 pouvait n'envoyer qu'une seule geometrie.
- Les elements `unconnected_items` sont compares par leur nombre avant/apres,
  car KiCad change leur identite JSON lors de la reconstruction des pistes.
  Les autres categories DRC restent comparees exactement.
- Le diagnostic affiche les nombres d'elements non connectes avant et apres
  lorsqu'ils sont presents ou en augmentation.
- Aucune modification IPC n'est incluse.

Validation volontairement limitee aux controles unitaires ciblant le
portefeuille unitaire, le comparateur DRC et le portefeuille natif. Aucun SET21
ni test de non-regression complet n'est execute.
