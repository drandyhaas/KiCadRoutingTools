# KiCad Track Gloss 1.2.1

Etat : version de test plugin, sans campagne complete de non-regression.

- KiCad 10 reste l'unique API cible. Aucune migration IPC n'est incluse.
- Le gloss peut couper localement un angle interne a 90 degres en conservant
  les deux portions de piste et en agrandissant le chanfrein jusqu'a la limite
  sure.
- Les chanfreins locaux participent au meme best-of que les raccourcis et les
  glissements de terminaison existants.
- Le plugin converge jusqu'au point fixe ou au budget temporel, sans plafond
  global de quatre passes.
- En multi-net, les plans de connexions sont prepares avant le plan global afin
  de disposer rapidement de candidats recuperables.
- Le recuperateur DRC valide trois candidats en parallele dans une meme vague
  (portefeuille 3/3).
- Les protections restent limitees a l'autorite native de KiCad et chaque plan
  retenu reste soumis au DRC natif.

Validation volontairement limitee a des controles unitaires cibles et au seul
PCB magic_keys. Aucun SET21 ni test de non-regression complet n'est execute.

Resultats magic_keys avec un minimum de 0,2 mm et DRC natif propre :

- connexion VCC unitaire : 14,028645 mm economises contre 10,376483 mm en
  version 1.1.1 (+35,2 % sur le gain) ;
- selection ALL, budget 60 s, portefeuille DRC 3/3 : 308,090444 mm economises,
  104 connexions conservees sur 113, en 54,9 s ;
- a budget comparable, la precedente strategie DRC 2/2 avait obtenu
  277,150586 mm et 98 connexions : le 3/3 ajoute 30,939857 mm (+11,2 % sur le
  gain) et six connexions.
