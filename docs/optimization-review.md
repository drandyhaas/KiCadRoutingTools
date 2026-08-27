# Revue d'optimisation temporelle

Date de mesure : 2026-08-27
Branche de travail : `optimize`
KiCad mesure : 10.0.5 (`D:\kicad\bin`)
Fixture : `tests/track_gloss/patterns/dispenser_labels/dispenser_labels.kicad_pro`

## Conclusion

L'objectif de reduction de 50 % est atteignable pour le CLI `ALL`, mais pas
en optimisant indistinctement tout le planificateur. Deux recalculs dominent :

1. le seed de clearance netclass relance recursivement toute la recherche sur
   chaque passe, meme si seuls quelques groupes net/couche peuvent differer ;
2. apres un rejet DRC, le fallback conservateur est reconstruit depuis zero
   alors que son etat de premiere passe existait deja pendant la convergence.

La cible propre est donc une recherche par groupes avec deux seeds seulement
pour les groupes concernes, qui retourne des le premier calcul une echelle de
candidats `{raffine, premiere_passe}`. Cette architecture devrait depasser
50 % de gain sur le fixture tout en conservant l'objectif lexicographique
actuel : corrections non octolineaires, cuivre economise, segments retires.

Pour un net unique, le diagnostic est different : le DRC natif occupe 84 % du
temps. Avec les API publiques de KiCad 10, une reduction de 50 % n'est pas
possible sans changer explicitement le contrat de validation, ou sans faire
evoluer KiCad avec un DRC cible/incremental.

## Mesures de reference

| Cas | Temps observe | Planification | DRC natif | Lecture |
| --- | ---: | ---: | ---: | --- |
| CLI `ALL`, 590 pistes eligibles | environ 54 s | environ 51 s | 2,16 s | l'algorithme domine |
| CLI `net:/cpu/tx`, 10 pistes | 2,62-2,68 s | 0,241 s | 2,24 s | le DRC represente 84 % |

Le profil complet instrumente est volontairement plus lent que le temps mur
normal : 354 366 844 appels en 115,789 s. Il reste utile pour la repartition :

- `generate_converged_plan`: 110,913 s cumules, deux appels ;
- `generate_candidate_plans`: 107,712 s cumules, 12 appels dont 6 recursifs ;
- `smooth_selected_chains`: 88,846 s, 384 appels ;
- `path_blocker`: 73,148 s, 223 557 appels ;
- `_capsule_interval`: 26,972 s, 71 248 appels ;
- validation de connectivite: 3,255 s ;
- les attentes de workers: 16,960 s.

Le profil net unique donne 2,682 s, dont 2,241 s dans
`validate_native_plan`, 0,241 s dans deux appels de convergence et 0,247 s
pour construire le snapshot candidat. Le second appel de convergence est le
fallback declenche apres le rejet DRC.

### Experience sur le seed netclass

Une execution de planification seule a compare le comportement actuel au meme
appel avec `_allow_netclass_seed=False` :

| Variante | Temps | Passes | Corrections d'angle | Cuivre gagne | Retire/ajoute |
| --- | ---: | ---: | ---: | ---: | ---: |
| seed actuel | 31,87 s | 3 | 40 | 64,580676 mm | 211/182 |
| sans second seed | 9,61 s | 2 | 39 | 64,711296 mm | 211/183 |

La suppression brute accelere la recherche de 69,8 %, mais perd une correction
non octolineaire. Elle ne doit donc pas etre livree telle quelle. Le fixture
contient 52 groupes net/couche ; seuls 7 groupes, soit 313 pistes, satisfont les
conditions `mixed_width && resolved_differs`. Le second seed doit etre local a
ces groupes, puis compose avec les resultats ordinaires des 45 autres groupes.

## Revue detaillee de l'algorithme de decoupe

### Etat actuel

La decoupe est repartie entre plusieurs representations implicites :

- `_canonicalize_eligible_subdivisions` fusionne des morceaux collineaires ;
- `smooth_selected_chains` reconstruit une incidence locale et marche depuis
  les ancres ;
- `_junction_branch_scopes` reconstruit une autre incidence pour les T ;
- `find_track_terminal_targets` et `find_pad_terminal_targets` rescannent les
  terminaisons ;
- la convergence applique, refusionne, trie et recalcule une signature de
  toute la carte a chaque passe ;
- les groupes largeur/net/couche sont encore reconstruits dans la generation
  des candidats.

Chaque fonction est raisonnablement conservative prise separement, mais leur
composition rend la decoupe difficile a prouver et multiplie les parcours.

### Points a corriger

1. **Topologie non partagee.** Les sommets, incidences, ancres, branches et
   groupes doivent provenir d'un seul `TopologyIndex` immuable. Aujourd'hui,
   des fonctions voisines peuvent prendre des decisions avec des cles de
   sommet ou des tolerances differentes.

2. **Coordonnees brutes contre sommets normalises.** La canonicalisation place
   les endpoints bruts dans `endpoint_buckets`, alors que la construction de
   chaine utilise `vertex(...)=round(..., 6)`. Deux points electriquement
   equivalents peuvent donc etre joints dans une phase et separes dans une
   autre. Une unique fonction de quantification doit fournir la cle de sommet,
   tout en conservant les coordonnees originales pour la geometrie exacte.

3. **Limite cachee de 100 segments.** La marche de chaine s'arrete a
   `len(chain) >= 100`. Le reste est traite depuis l'autre ancre comme une
   autre chaine et une simplification traversant cette coupure devient
   impossible. Cette borne doit etre remplacee par une detection de cycle
   explicite et un budget porte par le nombre de spans/candidats, observable
   dans les statistiques.

4. **Recherche de terminaux par scans.** `find_track_terminal_targets` compare
   chaque endpoint eligible a toutes les pistes immuables ; la variante pad
   parcourt toutes les regions de pad. Ces recherches doivent interroger les
   index spatiaux deja construits dans `PlannerContext`.

5. **Convergence globale apres changement local.** Les signatures trient toute
   la geometrie de carte et les passes reconstruisent des contextes logiques
   alors que seuls quelques groupes ont change. La prochaine passe doit porter
   une file de groupes sales : groupe modifie et groupes voisins dans son
   enveloppe de clearance. La signature de cycle peut etre composee a partir
   des signatures par groupe.

6. **Fallback netclass recursif et global.** `generate_candidate_plans`
   rappelle actuellement `generate_candidate_plans` sur un modele ou toutes
   les clearances de segments valent `-1`. Il faut produire les deux variantes
   au niveau du groupe concerne, partager topologie et geometries candidates,
   puis choisir selon le meme objectif lexicographique.

7. **Fallback DRC recalcule.** `generate_converged_plan` connait deja le plan
   compose apres la premiere passe. Il doit le conserver comme candidat
   conservateur. Le workflow ne doit jamais rappeler le planificateur apres un
   rejet natif pour retrouver un etat deja visite.

8. **Test numerique couteux de capsule.** `_capsule_interval` effectue jusqu'a
   64 iterations ternaires puis deux fois 64 dichotomies. Il represente 27 s
   sous profil. Une intersection analytique segment/capsule est preferable.
   Une etape moins risquee consiste a memoiser les couples geometrie/rayon,
   mesurer le taux de hit, puis remplacer le solveur avec des tests de
   propriete contre l'implementation actuelle.

9. **Validation composee trop tardive.** `validate_result` reconstruit encore
   des partitions apres composition. Le nouvel index topologique doit fournir
   une signature de connectivite de reference par net et recalculer uniquement
   les composantes touchees par les suppressions/additions.

### Decoupe cible

Le pipeline propose est le suivant :

1. construire une fois `BoardContext` : objets, regles resolues et index
   spatiaux globaux ;
2. construire `TopologyIndex` par `(net_id, layer)` avec sommets quantifies,
   incidences et raisons d'ancrage ;
3. condenser chaque composante collineaire degree-2 en `CanonicalRun`, avec la
   liste reversible de ses segments natifs ;
4. extraire des `ChainView` entre ancres sans limite arbitraire ;
5. enumerer une seule fois les spans et paires de terminaux, avec caches de
   geometrie independants de la strategie ;
6. resoudre chaque chaine par programmation dynamique ;
7. resoudre chaque groupe avec le seed normal et, seulement si necessaire, le
   seed netclass ;
8. composer les groupes en conservant les conflits explicites ;
9. valider incrementalement les nets affectes ;
10. pousser seulement les groupes sales dans la passe suivante ;
11. retourner ensemble le plan raffine et le plan conserve de premiere passe.

Cette structure retire les notions concurrentes de « subdivision », « chain »,
« branch scope » et « fallback group » au profit de vues d'une meme topologie.

## DRC et gloss d'un net unique

### Limite de l'API KiCad 10

Le CLI officiel `kicad-cli pcb drc` ne propose aucun filtre par net, couche,
boite ou liste d'objets. Il execute un DRC de carte. Dans la source KiCad 10,
`DRC_ENGINE::RunTests` invalide les caches de carte, regenere le cache DRC puis
parcourt tous les providers. `DRC_ENGINE` n'est par ailleurs pas expose par le
module SWIG public ; seul `WriteDRCReport` apparait, et son utilisation exige
un contexte applicatif KiCad complet absent du Python headless autonome.

Sources :

- [documentation officielle du CLI KiCad 10](https://docs.kicad.org/10.0/en/cli/cli.html#pcb-drc) ;
- [source officielle de `DRC_ENGINE::RunTests`](https://gitlab.com/kicad/code/kicad/-/raw/10.0/pcbnew/drc/drc_engine.cpp) ;
- [interface officielle `DRC_ENGINE`](https://gitlab.com/kicad/code/kicad/-/raw/10.0/pcbnew/drc/drc_engine.h) ;
- [helper Python officiel `WriteDRCReport`](https://gitlab.com/kicad/code/kicad/-/raw/10.0/pcbnew/python/scripting/pcbnew_scripting_helpers.cpp).

Supprimer `--refill-zones` n'apporte rien sur le fixture : 1,538 s sans refill
contre 1,517 s avec refill sur les essais ponctuels. Changer le niveau de
severite ne change pas non plus le cout du moteur (environ 1,54-1,59 s), car il
filtre surtout le rapport, pas les providers executes.

### Defaut de stabilite a traiter avant l'optimisation

Trois DRC successifs du meme fichier, avec refill, ont retourne :

- clearance : 19, 16, 16 ;
- hole-clearance : 12, 13, 13.

Trois executions sans refill ont egalement varie : clearance 18, 15, 17 et
hole-clearance 9, 12, 13. Le comparateur par empreintes classe alors plusieurs
resultats du meme fichier comme des augmentations. L'execution concurrente
avant/apres peut donc produire des faux rejets. Une fixture identique
baseline/candidate doit etre ajoutee comme test de stabilite repete avant toute
modification de performance.

Ce point ne justifie pas de rendre le comparateur permissif : il faut d'abord
capturer les rapports complets, determiner si les constats instables touchent
des UUID identiques et isoler l'effet KiCad. En cas d'instabilite native
confirmee, la politique doit etre explicite : retry borne et consensus, ou
validation des seuls constats impliquant la geometrie modifiee avec preuve de
couverture pour les suppressions.

### Options realistes

1. **Mode complet actuel, plus propre.** Conserver deux DRC en parallele et
   reutiliser le cache baseline en session. Cela preserve le contrat, mais ne
   peut pas atteindre -50 % sur un net unique car le DRC candidat reste sur le
   chemin critique.

2. **Entree attestee propre + DRC candidat seul.** Un nouveau contrat CLI peut
   accepter une preuve amont que l'entree a passe le meme KiCad/rules/hash.
   Cela evite le DRC baseline, mais le temps estime reste proche de 2 s : gain
   insuffisant pour -50 %.

3. **Mode `trusted-internal` explicite.** Pour un pipeline qui execute deja son
   propre DRC apres scoring, sauter la porte native locale donne un plancher
   mesure proche de 0,44 s et depasse largement -50 %. Ce mode ne doit jamais
   etre silencieux ni devenir le defaut de l'ActionPlugin. Le JSON doit porter
   l'attestation amont, le mode de validation et `native_drc_gate:not_run`.

4. **Validation locale candidate-only.** Enrichir le helper d'application pour
   retourner les UUID des nouvelles pistes, puis ne comparer que les constats
   DRC qui impliquent un UUID ajoute ou la zone de connectivite affectee. Cette
   voie peut supprimer le baseline, mais pas le cout du DRC global ; elle
   ameliore surtout la stabilite et necessite une preuve particuliere pour les
   regressions causees par une suppression.

5. **Evolution KiCad.** La vraie solution sans compromis est un job/API DRC
   incremental acceptant les objets sales ou une region d'influence. C'est la
   seule voie pour conserver l'autorite native tout en visant -50 % sur une
   petite selection avec une grande carte.

## Plan d'implementation priorise

### Nouvelle priorite : validation DRC d'un portefeuille en une vague

Les mesures SET21 montrent qu'apres les optimisations du planificateur, les
deux validations natives sequentielles deviennent le principal plafond. Le
CLI valide d'abord le plan raffine, attend son rejet, puis valide le plan
conservateur deja conserve dans `conservative_ladder`. Sur les trois grandes
cartes mesurees, cette seconde vague coute environ 2,1 a 2,9 secondes.

La voie substantielle proposee est une API `validate_plan_ladder` qui :

1. sauvegarde et hache la baseline une seule fois ;
2. materialise le plan raffine et le plan conservateur dans deux snapshots ;
3. lance en parallele le DRC baseline et les deux DRC candidats ;
4. compare les deux rapports candidats a la meme baseline ;
5. retient le premier plan autorise dans l'ordre de qualite existant.

Le contrat de securite ne change pas : aucun candidat n'est applique sans son
DRC natif, et toute erreur ou expiration reste un rejet ferme. Le cout CPU et
la pression memoire augmentent ponctuellement, mais le chemin critique d'un
rejet passe de deux vagues DRC a une seule. Sur le dernier echantillon SET21 a
0,1 mm, la borne haute est la suppression d'environ 7,4 secondes sur 30,8,
soit 24 % supplementaires avant contention. Cette optimisation cible aussi le
plugin, dont le budget total est aujourd'hui consomme par la seconde vague.

Elle doit etre accompagnee d'une limite stricte a deux candidats, d'un budget
commun, de caches indexes par `(baseline_hash, plan_hash)`, et de tests qui
prouvent que le choix reste exactement `raffine si valide, sinon conservateur
si valide, sinon no-op`.

#### Etat realise

La validation de portefeuille est maintenant implementee avec une limite de
deux plans, une baseline partagee, un budget commun, les caches existants et
un rejet ferme par candidat. Le CLI et le plugin l'utilisent lorsque le plan
conservateur de premiere passe est deja disponible ; les petits scopes sans
ladder conservent le chemin sequentiel historique pour ne pas payer une
planification speculative.

Sur les quatre cartes SET21 au seuil partage de 0,2 mm, le temps mur cumule
passe de 31,186 s a 24,707 s, soit 20,8 % pour cette etape seule. Par rapport
a la baseline intermediaire originale de 39,531 s, le gain cumule atteint
37,5 %. La contention entre trois processus KiCad DRC explique l'ecart avec
la borne theorique. Les quatre plans, corrections d'angle et decisions DRC
sont identiques a la validation sequentielle a 0,2 mm.

La campagne finale de non-regression passe : 94 tests unitaires, sept ordres
de la selection complete identiques a 66,020888 mm, puis 334 scopes et 269
applications en memoire sans erreur.

### Nouvelle etape : convergence incrementale conservative

La passe suivante introduit un cache local de plans par groupe net/couche.
Son invalidation ne repose pas seulement sur le groupe : elle empreinte aussi
les terminaisons glissantes et tout cuivre present dans une enveloppe
d'influence elargie par les maxima globaux de largeur, clearance et rayon de
pad. Un changement hors de cette enveloppe laisse le groupe propre ; tout
changement dedans force son recalcul. Les pads, vias, keepouts et contours
restent constants pendant une convergence, et la composition conserve la
validation complete existante.

Cette etape vise surtout les cartes qui changent pendant deux ou trois passes :
les groupes deja fixes ne sont plus renvoyes aux processus de planification a
la passe suivante. Le cache est volontairement limite a un seul appel de
`generate_converged_plan`, distingue le mode d'ouverture du mode converge et
ne modifie ni l'ordre lexicographique ni le plan stocke.

En parallele, le DRC de fallback speculatif est maintenant annule des que le
plan principal est accepte. Les erreurs d'infrastructure DRC deviennent
fatales dans le CLI et l'erreur du fallback n'est plus masquee par un rejet
ordinaire du plan principal.

Conformement a la consigne de ce jalon, seuls les tests unitaires cibles et les
controles statiques sont executes. Aucune mesure SET21 ni campagne
d'anti-regression ne qualifie encore cette nouvelle etape.

### P0 - Mesure et correction

- ajouter au CLI les timings `load`, `snapshot`, `scope`, `planning_primary`,
  `planning_fallback`, `native`, `apply/save`, `total` ;
- etendre le benchmark corpus avec scopes repetables, warmups, repetitions,
  mediane/p95 et conservation des rapports DRC bruts ;
- ajouter le test DRC identique repete et caracteriser l'instabilite du fixture ;
- figer trois contrats de qualite : geometrie, corrections d'angle, cuivre et
  nombre de segments, pas uniquement le temps.

### P1 - Gains sans changement de qualite attendu

- conserver le plan de premiere passe dans le resultat de convergence et le
  reutiliser comme fallback DRC ;
- deplacer le double seed au niveau groupe et ne le lancer que sur les groupes
  `mixed_width && resolved_differs` ;
- partager le resultat de generation geometrique entre les deux evaluations
  de clearance ;
- memoiser `_capsule_interval`, mesurer hit-rate, memoire et determinisme.

### P2 - Refonte propre de la decoupe

- introduire `TopologyIndex`, `CanonicalRun`, `ChainView` et `DirtyGroupQueue` ;
- supprimer la limite de chaine 100 au profit de budgets observables ;
- remplacer les scans de terminaux par les index spatiaux ;
- rendre validation et signatures incrementales par net/groupe ;
- remplacer le solveur numerique de capsule par une version analytique prouvee
  par differential/property tests.

### P3 - Strategie net unique

- decider le contrat produit entre `full-native`, `attested-candidate` et
  `trusted-internal` ;
- ne proposer `trusted-internal` au CLI que si le pipeline amont fournit une
  validation native equivalente et traçable ;
- ouvrir en amont KiCad une demande/proposition de DRC incremental si le mode
  natif complet doit lui aussi atteindre -50 %.

## Criteres de sortie

Une optimisation CLI n'est retenue que si, sur le corpus complet :

- mediane et p95 baissent d'au moins 50 % pour `ALL` ;
- aucun cas ne perd de correction non octolineaire selon l'objectif actuel ;
- le cuivre gagne et le nombre de segments ne regressent pas a corrections
  egales ;
- la geometrie reste deterministe pour les permutations et subdivisions ;
- les tests unitaires, le run pattern, le full sweep et les comparaisons DRC
  passent ;
- les modes DRC et attestations sont visibles dans le JSON et ne peuvent pas
  etre confondus avec une validation native complete.
