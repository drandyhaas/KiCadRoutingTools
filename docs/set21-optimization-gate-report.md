# Rapport de jalon d'optimisation SET21

Date : 2026-08-27
Branche : `optimize`
Baseline : commit intermediaire `7df62886`
KiCad : 10.0.5 (`D:\kicad\bin`)
Decision actuelle : **PASS — gain cumule SET21 de 37,5 %**

## Etat de la nouvelle version non qualifiee

Une optimisation supplementaire est presente dans le working tree : cache
incremental conservatif des groupes propres entre les passes de convergence,
annulation du DRC de fallback quand le candidat principal est accepte et
propagation fatale des erreurs d'infrastructure DRC dans le CLI.

Ces changements sont uniquement verifies par des tests unitaires cibles et
des controles statiques. A la demande explicite de ce jalon, les tests
d'anti-regression et le corpus SET21 ne sont pas relances. Les chiffres de
37,5 % ci-dessous caracterisent donc l'etat precedent, pas encore cette
nouvelle version, et aucune archive de version n'est produite.

> Le premier jalon ci-dessous est conserve comme historique. Le travail a
> ensuite repris avec un seuil commun de 0,2 mm et la validation native de deux
> candidats en une seule vague.

## Reprise et resultat final

La baseline optimisee mais encore sequentielle a 0,2 mm prend 31,186 s sur les
quatre cartes. La validation DRC du portefeuille `{raffine, conservateur}`
reutilise une seule baseline et lance les deux candidats dans la meme vague.

| Carte | DRC sequentiel 0,2 mm | Portefeuille DRC 0,2 mm | Qualite / decision |
| --- | ---: | ---: | --- |
| `kivu12` | 2,881 s | 2,796 s | identique, accepte |
| `polykit_x_inputboard` | 6,246 s | 4,585 s | identique, fallback accepte |
| `led_ring_crossbar` | 11,163 s | 8,676 s | identique, rejete |
| `uncutgem_nv` | 10,896 s | 8,650 s | identique, rejete |
| **Total** | **31,186 s** | **24,707 s** | **identique** |

Cette etape gagne 20,8 %. Par rapport a la baseline originale du jalon
(`39,5307 s`), le gain cumule est de 37,5 %, donc superieur a la porte de 30 %.

La campagne de non-regression couteuse n'a ete lancee qu'une fois, sur l'etat
final :

- 94 tests unitaires passes ;
- sept permutations de l'ordre des pistes donnent 66,020888 mm ;
- full sweep : 706 pistes, 334 scopes, 269 changements et 269 applications
  en memoire reussies.

Aucune version ni archive PCM n'a ete produite et les optimisations ne sont
pas poussees.

## Correctif DRC intermediaire

La variabilite des rapports DRC etait causee par la suppression des constats
repetes par piste lorsque `--all-track-errors` n'est pas fourni. Les providers
DRC internes s'executent concurremment ; le constat retenu pour une grappe de
collisions pouvait donc changer entre deux snapshots identiques.

Avec `--all-track-errors`, quatre executions successives donnent les memes
comptes :

- `clearance`: 25 a chaque execution ;
- `hole_clearance`: 17 a chaque execution ;
- aucune augmentation selon le comparateur avant/apres.

La seule empreinte variable restante appartient a `unconnected_items`, deja
comparee par delta de compte plutot que par identite. Le correctif et son test
cible ont ete pousses sur `origin/optimize` au commit `7df62886`.

## Optimisations experimentees apres le correctif

- conservation du plan d'ouverture des grandes selections pour eviter de
  reconstruire le fallback conservateur apres un rejet DRC ;
- memoisation bornee des intersections segment/capsule ;
- memoisation bornee des resultats de collision pour un modele immuable ;
- recherches de terminaisons track/pad via les index spatiaux existants ;
- suppression de la coupure artificielle des chaines a 100 segments ;
- ajout au JSON CLI des temps par phase et de la qualite du plan avant DRC ;
- extension du driver de benchmark avec qualite, planning et DRC separes.

Le fixture principal `dispenser_labels` passe d'environ 54 s a 29,315 s, soit
environ 45,7 % de gain, avec les memes 40 corrections d'angle et la meme
convergence. Ce resultat n'est toutefois pas representatif du petit echantillon
SET21, car cette carte active le fallback netclass global couteux.

## Echantillon SET21

Les quatre sources et leurs projets associes proviennent du manifeste
`tests/stress/manifest_set21.json`. Les deux versions ont utilise les memes
fichiers bruts et KiCad 10.0.5.

| Carte | Baseline CLI | Optimise CLI | Gain | Qualite du plan |
| --- | ---: | ---: | ---: | --- |
| `kivu12` | 2,8179 s | 2,9141 s | -3,4 % | identique |
| `polykit_x_inputboard` | 7,6634 s | 6,4920 s | 15,3 % | identique |
| `led_ring_crossbar` | 15,1500 s | 11,3696 s | 25,0 % | identique |
| `uncutgem_nv` | 13,8994 s | 11,0338 s | 20,6 % | identique |
| **Total** | **39,5307 s** | **31,8095 s** | **19,5 %** | **identique** |

La qualite a ete comparee avant la porte DRC sur :

- longueur planifiee economisee ;
- corrections non octolineaires ;
- segments retires et ajoutes ;
- nombre de passes et atteinte du point fixe.

Toutes ces valeurs sont identiques entre baseline et version optimisee sur les
quatre cartes.

Le planificateur seul gagne seulement 4,7 % en cumul sur cet echantillon
(13,0227 s vers 12,4076 s). Aucune des quatre cartes n'active le double seed
netclass ; l'essentiel du gain CLI vient donc de la reutilisation du fallback
apres un rejet DRC. Sur la petite carte `kivu12`, le DRC domine et sa variance
efface le faible gain du planificateur.

## Decision du jalon

Le gain cumule SET21 de 19,5 % est inferieur au seuil de 30 %. En consequence :

- la suite complete de non-regression n'a pas ete lancee ;
- aucune version ni archive PCM n'a ete produite ;
- les optimisations restent dans le working tree de `optimize`, non poussees ;
- seul le correctif DRC intermediaire `7df62886` est publie.

## Prochaine piste si le travail reprend

Les caches et la reutilisation du fallback ne suffisent pas sur les cartes qui
n'activent pas le fallback netclass. Pour depasser 30 % sur SET21, il faut une
reduction structurelle du premier calcul : topologie partagee, enumeration des
spans une seule fois, file de groupes sales entre les passes et solveur
analytique segment/capsule. Ces changements demandent une campagne de qualite
plus large avant d'etre retenus.
