# CONTEXT PROJET NJORD — volet GUI de mission

> Ce fichier est le volet **GUI** du document de passation. Il est destiné à
> être fusionné avec le `CONTEXT_PROJET_NJORD.md` de `node-ros-2026`.
> Dernière mise à jour : stage 5 terminé.

## Ce qui a été construit

Une interface de mission temps réel pour l'Asket, pour la campagne **Namibie**
(Cerulean Omniscan 3D), plus les paquets ROS 2 qui l'alimentent.

L'interface tourne dans un navigateur sur un portable à terre. Elle affiche
l'état du bateau, la navigation, les obstacles, la santé des capteurs,
l'enregistrement de mission et les commandes de mode.

**Ce n'est pas** un outil d'analyse sonar : pas de nuage de points, pas de
maillage, pas de bathymétrie. C'est le travail de SonarView, après la mission.

**Ce n'est pas** dans la chaîne de sécurité. Le coupe-circuit matériel et la
voie 8 de la radiocommande sont souverains ; rien ici ne peut les contourner,
les retarder ni les gêner. Voir `docs/safety.md`.

## Paquets ajoutés

| Paquet | Rôle |
|---|---|
| `asket_common` | Géométrie, plan de levé en tondeuse, qualité de cap |
| `asket_interfaces` | Messages et services |
| `asket_sim` | Sources simulées de tout, avec injection de pannes |
| `omniscan_bridge` | Cerulean Ping Protocol → ROS 2 |
| `mission_recorder` | Fichiers de mission, trajectoire, export vérifié |
| `system_test` | Test intégré avant mise à l'eau, verdict GO / NO-GO |
| `gui_backend` | FastAPI + WebSocket, négociation de débit |
| `asket_gui` | Frontend React + MapLibre |
| `asket_bringup` | Fichiers de lancement |

Aucun paquet existant (`pico_bridge`, `boat_bt`, Nav2, MAVROS, `rplidar`,
`balise_bridge`) n'a été modifié.

## Pour démarrer

```bash
pytest                                     # toute la suite, sans ROS
ros2 launch asket_bringup sim.launch.py    # tout le système, sans matériel
python3 -m gui_backend.core.app --sim      # juste l'interface, sur un portable
```

## Décisions structurantes

**La simulation est permanente, pas un échafaudage.** Aucun matériel n'était
connecté pendant le développement, et elle reste utile ensuite : développement
sans bateau, tests de non-régression, et à terme rejeu de mission.

**Le sonar simulé émet de vraies trames Ping Protocol sur une vraie socket
UDP.** `omniscan_bridge` est donc identique en simulation et sur le terrain :
pas de second chemin de code qui pourrit sans que personne s'en aperçoive.

**Le backend n'envoie rien par défaut.** Le client s'abonne à des flux nommés,
le serveur décide de ce qu'il envoie réellement et *dit pourquoi*. C'est la
seule défense structurelle contre une interface parfaite au banc qui s'effondre
à 200 m du bord.

**L'état affiché est toujours l'état confirmé.** Une commande a trois issues et
trois seulement : `pending`, `confirmed`, `failed`. La confirmation vient de ce
que le Pico rapporte, jamais d'un accusé de réception.

**Chaque valeur porte son âge**, calculé contre l'horloge du *serveur*. Une
valeur périmée est barrée en rouge : sur une liaison intermittente, un
opérateur ne doit jamais prendre une position ancienne pour une position
actuelle.

## Points d'attention pour la suite

1. **La disposition binaire de `OS3D_POINT_SET` est transcrite depuis le
   cahier des charges, pas depuis une documentation Cerulean.** Le parseur
   recoupe le nombre de points annoncé avec la taille réelle et signale un
   écart, donc une erreur échoue bruyamment. **À valider sur les données
   d'exemple Cerulean avant le premier déploiement.** (Question 8)

2. **Le bras de levier entre l'antenne GNSS et le transducteur doit être mesuré
   au centimètre.** C'est un décalage systématique qu'aucun post-traitement ne
   retrouvera. Valeur provisoire dans `omniscan_bridge/config/mounting.yaml`.
   (Question 2)

3. **L'horloge du sonar est le point de rupture silencieux.** Toute la fusion
   post-mission repose sur `utc_msec`. Si elle dérive et que personne ne le
   voit, la campagne n'est pas dégradée : elle est perdue, et on l'apprend de
   retour à Windhoek. Elle est surveillée en continu, remontée en alarme, et
   journalisée chaque seconde dans `diagnostics.jsonl`.

4. **Le message réel de `pico_bridge` n'est pas connu ici.** Le backend passe
   par `gui_backend/config/topics.yaml` : le brancher sur le vrai message est
   un changement de configuration plus une fonction d'adaptation.
   (Question 7)

5. **Le test moteur n'est volontairement pas branché.** La barrière logicielle
   existe et est testée ; la relier au Pico appartient au jour où quelqu'un est
   debout à côté du bateau.

Les six questions ouvertes du cahier des charges, plus trois soulevées pendant
le développement, sont suivies dans `docs/open_questions.md`, avec les valeurs
provisoires utilisées en attendant. Elles sont toutes marquées `PROVISIONAL`
dans le fichier de configuration qui les porte :

```bash
grep -rn PROVISIONAL src/
```

## Ce qui n'est pas fait

Test moteur actif branché, suivi et scoring de confiance des obstacles,
planificateur de lignes dans l'interface, sortie NMEA pour SonarView en direct,
RTK/NTRIP pour le Njord Challenge.
