# KiCad Track Gloss 1.2.3

Etat : version intermediaire de diagnostic pour tests unitaires dans KiCad,
sans campagne complete de non-regression.

- Apres ecriture, le plugin relit le cuivre de la carte active en unites
  internes KiCad.
- Il verifie que chaque piste demandee en suppression a disparu et que chaque
  piste ajoutee existe avec les extremites, largeur, couche et net attendus.
- Un ecart de relecture fait echouer l'operation et active le rollback au lieu
  d'annoncer `GLOSS APPLIED`.
- Le rapport de diagnostic confirme une application correcte par
  `Post-apply copper readback: requested plan matched.`
- Le portefeuille unitaire et le comparateur DRC introduits en 1.2.2 sont
  conserves.
- Aucune modification IPC n'est incluse.

Cette version ne contient pas encore la generalisation du moteur aux
translations interieures et aux sequences de plusieurs chanfreins. Elle sert a
isoler sans ambiguite le comportement d'application avant cette evolution.

Validation volontairement limitee aux tests ciblant la relecture native et les
controles du portefeuille 1.2.2. Aucun SET21 ni test complet de non-regression
n'est execute.
