# Prompt de reconstruction de KiCad Track Gloss

Copier le texte compris entre les deux séparateurs dans une nouvelle tâche
ChatGPT/Codex ayant accès au dépôt de travail, aux exécutables KiCad et aux
cartes de test. Adapter uniquement les chemins locaux indiqués au début.

---

Tu es responsable de concevoir, implémenter, tester et empaqueter **KiCad Track
Gloss**, un ActionPlugin autonome pour KiCad 10. Travaille directement dans le
dépôt fourni et livre un résultat utilisable, pas seulement une proposition.

## Contexte à renseigner

- Dépôt de travail : `<REPOSITORY_PATH>`
- Interpréteur Python de KiCad : `<KICAD_PYTHON_PATH>`
- Exécutable `kicad-cli` : `<KICAD_CLI_PATH>`
- Corpus de cartes de stress, s'il est disponible : `<STRESS_CORPUS_PATH>`
- Branche de départ : la branche contenant les travaux de
  `fca1/KiCadRoutingTools`, sans fusionner le plugin vers `main`.

Commence par inspecter le dépôt, son état Git, ses instructions locales et le
code de `origin/main`. Préserve toutes les modifications utilisateur sans
rapport avec cette tâche. Utilise autant que possible le code, les idées et les
outils existants de KiCadRoutingTools au lieu de réinventer leurs concepts.

## Produit attendu

Le plugin raccourcit et simplifie des pistes déjà routées. L'utilisateur
sélectionne un ou plusieurs segments droits dans PCB Editor puis lance le
gloss. Le plugin étend automatiquement chaque segment à sa connexion utile,
cherche une géométrie plus courte ou plus simple constituée exclusivement
d'angles 0/45/90 degrés, valide le résultat et l'applique directement au PCB
courant en une seule opération Undo.

L'utilisation normale doit être fluide : aucune prévisualisation, aucune
sauvegarde avant/après, aucune confirmation après réussite et aucun message de
succès. Si le résultat ne convient pas, l'utilisateur emploie Undo. Un no-op
valide déclenche uniquement l'avertissement sonore standard de KiCad. Une
erreur interne inattendue doit, elle, produire un rapport exploitable.

Le résultat doit comprendre :

1. un ActionPlugin normal silencieux ;
2. un ActionPlugin de diagnostic utilisant exactement le même moteur ;
3. un moteur géométrique indépendant de `pcbnew` ;
4. une couche d'adaptation KiCad étroite ;
5. un CLI headless utilisant le même algorithme et compatible avec
   `place_route_loop --accept-cmd` ;
6. une configuration interne JSON avec valeurs sûres par défaut ;
7. des tests unitaires, des patterns KiCad réels et un paquet PCM installable.

## Contrat d'utilisation du plugin

- Accepter un ou plusieurs segments droits, éventuellement mélangés dans la
  sélection avec des vias, empreintes, textes ou autres objets. Filtrer les
  objets étrangers sans erreur.
- Plusieurs segments peuvent appartenir à plusieurs nets. Les traiter comme
  un lot déterministe, sans que l'ordre de sélection, l'ordre des nets ou
  l'ordre des objets dans le fichier change le résultat.
- Un seul segment sélectionné sert de graine : reproduire automatiquement
  l'effet de **Select/Expand Connection** jusqu'aux terminaisons et obstacles
  pertinents.
- Ne modifier que le cuivre autorisé par les graines sélectionnées. Les pistes
  hors scope, pads, vias, arcs, pistes verrouillées, paires différentielles
  probables et méandres d'accord de longueur restent protégés.
- Si aucun segment droit n'est sélectionné, ouvrir la boîte de paramètres de
  session au lieu de lancer le gloss. Son titre contient la version. **Close**
  conserve les valeurs en mémoire jusqu'à la fermeture de KiCad ; **Cancel**
  ne les applique pas. Aucun fichier de configuration n'est réécrit.
- Afficher en première ligne le choix du DRC natif pour une sélection d'une
  piste et indiquer clairement que le désactiver rend l'interaction beaucoup
  plus rapide mais supprime cette validation native.
- Le minimum de gain est éditable par pas de 0,1 mm. Les limites de passes
  restent internes et ne sont pas affichées. Chaque champ visible possède une
  infobulle claire.
- Ne passer au curseur occupé que si la planification dépasse trois secondes.
  Les lectures et écritures `pcbnew` restent sur le thread principal ; seul le
  planificateur API-neutre peut travailler en arrière-plan.
- Le diagnostic a trois onglets : **Result**, **Details** et **JSON**. Le
  premier donne immédiatement le gain en mm et les segments avant/après. Le
  second explique scope, protections, transformations, rejets, nets bloquants,
  convergence et temps. Le troisième contient uniquement le JSON. Ajouter
  **Copy tab** et **Copy all**.

## Règles du moteur

### Géométrie et objectif

- Produire uniquement des segments 0/45/90 degrés. La correction d'un segment
  non octolinéaire est prioritaire et peut accepter une très faible hausse de
  longueur si elle est indispensable à la normalisation.
- Entre solutions octolinéaires sûres, maximiser d'abord la longueur de cuivre
  économisée, puis la réduction de segments, puis utiliser une signature
  géométrique stable pour départager.
- Utiliser les coordonnées exactes du cuivre existant et non la grille active
  de PCB Editor.
- Conserver net, couche et largeur. Les transitions entre largeurs peuvent se
  déplacer avec la géométrie ; ne jamais supposer que deux largeurs différentes
  sont un bruit négligeable ni introduire de seuil implicite dépendant de leur
  différence.
- Le résultat doit être invariant au découpage artificiel d'une ligne en
  plusieurs segments colinéaires. Fusionner/simplifier la représentation
  logique avant évaluation sans perdre les identités nécessaires à Undo et à
  la validation.

### Connexions et terminaisons mobiles

- Détecter les chaînes par net et couche, leurs branches, pads, vias, arcs et
  jonctions.
- Une terminaison arrivant au milieu d'une piste immuable du même net est une
  terminaison en T glissante. Le point de contact peut se déplacer le long de
  la piste traversante afin de trouver la connexion octolinéaire la plus courte
  et la plus utile. La piste traversante ne doit pas être modifiée hors scope.
- Une terminaison dans le cuivre d'un pad du même net peut glisser dans la zone
  réellement conductrice du pad. Prendre en compte cercle, rectangle, ovale,
  rectangle arrondi, rotation et couches.
- Pour un pad `CUSTOM`, utiliser la géométrie de cuivre effective fournie par
  KiCad, déjà unionnée, et non une chaîne de texte ni une simple bounding box.
  Conserver un fallback conservateur uniquement pour une forme réellement non
  prise en charge.
- Reconnaître les intersections same-net nouvellement créées comme des T et
  supprimer les queues libres devenues inutiles, sans supprimer une
  continuation utile.
- Un scope plus grand ne doit pas faire disparaître une bonne solution sûre
  disponible pour un sous-scope. Pour les petits scopes, conserver plusieurs
  alternatives locales aux jonctions et composer la meilleure combinaison.

### Recherche et convergence

- Générer des raccourcis octolinéaires déterministes et sélectionner des
  transformations compatibles globalement, par exemple avec une planification
  d'intervalles pondérés par chaîne.
- Rechercher à nouveau après chaque passe les simplifications ouvertes par la
  passe précédente. Composer toutes les passes contre l'état initial et
  appliquer un seul plan atomique.
- Détecter le point fixe. Avoir un garde-fou de passes distinct pour le plugin
  interactif et le CLI hors ligne. Ne jamais annoncer un point fixe lorsque la
  limite ou le budget temps a été atteint.
- Le plugin doit privilégier la réactivité et peut appliquer uniquement une
  amélioration partielle déjà entièrement validée. Le CLI peut attendre plus
  longtemps et vise le point fixe complet.
- Paralléliser uniquement les groupes net/couche réellement indépendants et
  seulement au-dessus d'un seuil où le démarrage de processus est rentable.
  Aucun objet `pcbnew` ne traverse un worker. Trier et sérialiser les résultats
  afin que parallèle et séquentiel donnent le même plan. Sur erreur de worker,
  revenir au séquentiel.

## Sécurité et intégration KiCad

Utiliser les API sémantiques de `pcbnew` pour les couches de cuivre, règles,
objets, connectivité, ajouts et suppressions. Ne jamais déduire une couche à
partir d'un suffixe textuel comme `.Cu` et ne jamais coder en dur `F.Cu` ou
`B.Cu` lorsque KiCad fournit l'identifiant ou l'appartenance cuivre.

Les validations rapides doivent couvrir au minimum :

- identité et autorisation de chaque suppression ;
- conservation du net, de la couche et de la largeur ;
- connectivité entre terminaisons immuables ;
- clearance piste/piste, piste/via et piste/pad ;
- pads traversants sur toutes leurs couches de cuivre ;
- vias traversants ;
- keepouts et rule areas ;
- géométrie complète d'Edge.Cuts, y compris arcs et trous internes ;
- graphiques explicites F.Mask/B.Mask à protéger ;
- règle effective fournie par KiCad et `.kicad_dru`, sans constante de
  clearance inventée.

Après composition, exécuter un unique garde DRC natif KiCad sur des snapshots
privés avant/après. Recharger et remplir les zones sur les copies avant le DRC.
Comparer les nouvelles violations de manière stable. Pour
`unconnected_items`, préférer l'augmentation de compte aux identités instables
produites par les refills. Ne modifier ni sauvegarder le PCB courant pendant
ce contrôle. Nettoyer les temporaires.

Sous Windows, lancer les helpers et `kicad-cli` sans fenêtre console et sans
entrée transitoire dans la barre des tâches. Mettre en cache de façon bornée un
baseline DRC uniquement si le contenu exact du PCB, du projet, du DRU, de la
version KiCad et des options de refill est identique. Ne pas transformer un
timeout ou une panne DRC en acceptation.

Autoriser un fast path sans DRC seulement lorsque l'absence de nouveau cuivre
et de zones rend mathématiquement la transformation sûre. Un simple déplacement
de coin, une terminaison mobile ou une coupe de trajectoire ne suffit pas.

Si le meilleur plan est rejeté par le DRC et que le budget restant le permet,
essayer une variante conservatrice distincte. Elle doit passer son propre DRC.

## Configuration interne initiale

Créer un JSON versionné et validé strictement, avec au minimum :

```json
{
  "schema_version": 1,
  "gloss": {"minimum_saved_length_mm": 0.1},
  "convergence": {
    "interactive_max_passes": 4,
    "interactive_group_max_passes": 2,
    "cli_max_passes": 16
  },
  "timing": {
    "interactive_total_time_budget_seconds": 10.0,
    "interactive_planning_time_budget_seconds": 5.0,
    "interactive_cancellation_grace_seconds": 1.0,
    "cli_total_time_budget_seconds": null
  },
  "safety": {"kicad_drc_for_single_track": true}
}
```

Ces valeurs reproduisent le comportement initial. Elles ne doivent pas être
dispersées sous forme de constantes concurrentes dans le code.

## CLI et compatibilité avec KiCadRoutingTools

Créer un CLI qui accepte directement un `.kicad_pcb` ou `.kicad_pro`. Par
défaut, il sélectionne toutes les pistes admissibles. Ajouter des scopes
explicites `ALL`, `net:<nom-exact>` et `segment:<uuid>`, répétables ou fournis
par manifeste JSON. Une exécution sans `--output` ne modifie jamais l'entrée.
Avec `--output`, écrire un nouveau PCB et refuser d'écraser l'entrée ; exiger
une option explicite pour remplacer un autre fichier existant.

Le CLI et le plugin doivent appeler le même moteur et la même orchestration de
convergence. Toutes les pistes sélectionnées dans le plugin doivent produire le
même plan que le scope `ALL` du CLI, hors différence explicite de budget temps.

Fournir `--max-passes`, `--time-budget`, `--no-parallel`, `--trace-passes`,
`--project`, `--output`, `--force` et un mode `--place-route-loop` acceptant les
trois chemins ajoutés par `place_route_loop`.

Le contrat stdout final est :

```text
SCORE_JSON={...}
GLOSS_SCORE_JSON={...}
SCORE=<float>
```

`SCORE_JSON=` est le contrat structuré canonique homogène avec les instruments
de notation KiCadRoutingTools. `GLOSS_SCORE_JSON=` en est un alias temporaire
de rétrocompatibilité portant exactement le même document. Ajouter
`--json-out` pour écrire ce payload sans préfixe dans un fichier. `SCORE=` est
toujours la dernière ligne, parsable par
`place_route_loop --accept-cmd`, et une valeur plus basse est meilleure. Le
score est la longueur totale virtuelle de pistes droites après gloss, en mm.
Les traces `GLOSS_PASS_JSON=` et les erreurs vont sur stderr. Un no-op valide
est un résultat normal ; une entrée invalide ou une panne est un code retour
non nul. Inclure dans le JSON le schéma, la version, les chemins, les scopes,
les longueurs, les segments, la convergence, le point fixe, le budget, le DRC,
les timings et l'éventuelle variante conservatrice.

Le score ne remplace pas le DRC, la connectivité, l'impédance ou les contraintes
fonctionnelles : le documenter comme une métrique de qualité complémentaire.

## Découpage attendu

Conserver une architecture compréhensible :

- `engine/model.py` : modèles immuables et plans ;
- `engine/geometry.py` : primitives sans dépendance KiCad ;
- `engine/context.py` : index et contexte partagé ;
- `engine/candidate_geometry.py` : sécurité géométrique locale exacte ;
- `engine/terminals.py` et `engine/pads.py` : T et pads mobiles ;
- `engine/planner.py` : chaînes, candidats, composition et convergence ;
- `engine/workflow.py` : stratégie partagée plugin/CLI et fallback ;
- `engine/parallel.py` : workers déterministes ;
- `engine/validation.py` : invariants avant application ;
- `engine/statistics.py` : métriques et catégories ;
- `kicad/reader.py`, `selection.py`, `rules.py`, `writer.py` : adaptation ;
- `kicad/native_validation.py` et `drc_report.py` : snapshots et DRC ;
- `kicad/settings_dialog.py`, `report_dialog.py`, `diagnostics.py` : interface ;
- `action_plugin.py` : orchestration légère ;
- une seule source de version partagée par l'interface et le packaging.

Éliminer les modules étrangers au plugin. Éviter les fichiers monolithiques,
les duplications plugin/CLI et les dépendances circulaires. Les textes visibles
par l'utilisateur et le diagnostic doivent être en anglais.

## Attribution et packaging

Ne pas présenter DrAndyHaas comme l'auteur du nouveau plugin. Conserver
clairement la provenance avec la formulation anglaise suivante ou son
équivalent fidèle :

> Inspired by and reusing part of DrAndyHaas's code.

Indiquer que l'adaptation autonome et ses modifications ont été produites avec
ChatGPT/Codex à la demande du responsable du projet, et que le projet est
maintenu par Frantz. Conserver la licence MIT, le copyright original applicable
au code repris et un `NOTICE` précis.

Produire une structure conforme au Plugin and Content Manager de KiCad :
`metadata.json`, icônes correctes, ActionPlugins enregistrés, versions
cohérentes, archive autonome sans modules inutiles, installation testable par
le manager et présence dans la barre d'outils ainsi que dans
**Tools > External Plugins**.

Créer également un répertoire `docs/` comparable à celui de KiCadRoutingTools,
avec des guides distincts `plugin-usage.md`, `cli.md`, `configuration.md`,
`output-contracts.md`, `safety-and-drc.md` et `architecture.md`. Le README
racine sert de page d'entrée courte et renvoie vers ces documents ; ne pas y
dupliquer toute la référence technique.

## Validation requise

Écrire des tests API-neutres rapides pour chaque invariant et des fixtures
KiCad réelles immuables. Couvrir au minimum :

- ordre de sélection et ordre des nets ;
- subdivision colinéaire d'un même trajet ;
- sélection unique contre sélection voisine plus large ;
- T glissant vertical, horizontal et diagonal ;
- glissement dans tous les types de pads, notamment `CUSTOM` ;
- largeurs différentes ;
- connexions multi-couches, vias et pads traversants ;
- Edge.Cuts avec arcs/trous ;
- `.kicad_dru` personnalisé ;
- meanders, paires différentielles, objets verrouillés et arcs protégés ;
- convergence, limite de passes et budget temps ;
- égalité séquentiel/parallèle ;
- plugin scope complet égal au CLI `ALL` ;
- stabilité des sorties JSON et `SCORE=` ;
- paquet PCM installable ;
- DRC KiCad avant/après sur plusieurs cartes réelles.

Les tests combinatoires très coûteux, notamment les permutations exhaustives
de sept ordres, ne doivent pas tourner à chaque itération. Prévoir un groupe
rapide courant et un groupe approfondi explicitement activé pour une release.
Le test d'invariance au découpage doit être exécuté au moins une fois avant la
release alpha, puis rester disponible sans alourdir chaque cycle.

Ne modifie jamais les cartes sources du corpus : travaille sur des copies et
publie les chemins des artefacts. Lors d'une validation DRC, comparer les
catégories et comptes avant/après, les connexions manquantes et quelques cas
représentatifs ; toute nouvelle régression bloque la release.

## Méthode de travail et livraison

1. Cartographier l'existant et écrire un plan court.
2. Reprendre le code utile de KiCadRoutingTools avec attribution.
3. Construire d'abord le modèle et les invariants, puis l'adaptation KiCad.
4. Ajouter les tests minimaux à chaque fonction critique.
5. Mesurer les performances séparément pour une connexion et pour un scope
   complet ; ne pas optimiser le gros lot au détriment du clic sur une piste.
6. Diagnostiquer les causes, pas seulement les symptômes, lorsque le résultat
   manuel est meilleur.
7. Nettoyer et découper le code après stabilisation.
8. Exécuter la suite rapide, puis les patterns réels et le DRC complet avant la
   release finale seulement.
9. Construire l'archive PCM, vérifier son contenu et fournir une synthèse avec
   version, tests, métriques, limites connues et chemins des artefacts.

N'affirme jamais qu'un test est passé sans l'avoir exécuté. Ne remplace pas une
API KiCad absente par une supposition : KiCad 10 n'expose pas publiquement son
optimiseur C++ PNS au plugin Python, donc utilise `pcbnew` pour les données et
les règles et garde le planificateur Python indépendant afin de pouvoir le
remplacer si une API officielle apparaît.

Le travail est terminé uniquement lorsque le plugin installé effectue le gloss
sur le PCB courant, que le diagnostic explique honnêtement le résultat, que le
CLI produit le même algorithme hors ligne, que les validations empêchent les
régressions DRC connues et que le paquet peut être testé par un utilisateur
sans intervention supplémentaire.

---
