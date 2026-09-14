# Connector geometry contract (#961)

The declared west/east/north/south board edge selects the drawn envelope's
support line in that outward direction. Signed position is positive outside;
body overhang is max(0, signed position), and setback is max(0, -signed position).
These are distances in mm at zero physical margin. A minimum overhang of zero
is nonbinding. Only an explicit max_setback_mm imposes seating. A connector
class does not impose a universal flush seat, centering, or maximum overhang.
The body drawing is a 2-D mechanical proxy, not a model of its 3-D mating cavity.

Supported: a closed convex polygonal envelope drawn on the footprint's own
F.Fab/B.Fab, else its own F.SilkS/B.SilkS. Lines, rectangles and polygons are
supported. Every convex-hull side must actually be drawn; internal markings are
allowed. Vertices are transformed at the actual arbitrary rotation before taking
extrema. Saved B-side local geometry is already mirrored. Stroke widths, text,
courtyards and pads are excluded. No pad box is substituted for a missing body.

Unsupported: open/clipped body marks, disconnected or concave body envelopes,
curves, opposite-face-only body drawings, and nonrectangular/curved/open board
boundaries, concavity and cutouts. These return null measurements and a reason.
Declared requirements left unmeasured prevent a complete passing floorplan
certificate, including an edge-only declaration. Rectangular outlines must be
proved from all source Edge.Cuts, not merely from bounding-box dimensions.
No local segment identifier exists yet for choosing an interior notch/cutout.

Copper edge gap is the minimum of all copper pads against all board edges;
the separate pad_copper_declared_edge_gap_mm is the declared-edge projection.
Copper shortfall and physical body overhang have independent dispositions.
The inherited analytic rectangular pad geometry and custom-rule coverage limits
remain explicit. Copper includes unconnected pads. Allowed body overhang never
waives copper containment/clearance, including seeder/reconstruction consumers.
The occupancy gate still uses max(clearance, board_edge_clearance); its sum is
an occupancy boundary shortfall, never physical body overhang.

Every edge_seating entry reports the measurements on passes and failures, with
mm units, geometry basis, limits, source and disposition. overhang_mm and
body_overhang_mm are the same value used by the clause. The legacy declared
field continues to mean an along-edge claim; body_requirement_declared is
separate. Along-edge positioning retains its existing occupancy-centre/span
contract, explicitly named in basis, independently of the body support line.

The oob_count exemption requires measured body geometry within the finite band,
any declared setback, and no crossing of another boundary. It only subtracts a
reference actually counted by the occupancy census; copper stays independently
reported. The legacy oob_amount budget remains the summed occupancy shortfall.
Emission uses this same zero-margin body geometry for observed bands and unique
edge inference, refuses ambiguous inference, and emits no class-derived maximum
or implicit seating limit. Observed bands remain labelled suggestions for editing.
An emitted intent is not an engineering-clean certificate.

Placement APIs retain their existing certification scopes: place_pose's
--strict-legal checks supported copper/hole/outline channels and takes no intent.
Final mechanical intent is certified separately with check_floorplan. Provenance
CLEAN means the accepted written pose is accounted for, not engineering-clean.
No-worse improving-pile publication remains available under inherited policy.

Integration base: 3f343981ae3fdbc82a60c0d41d693a03ccca98de.
Fetched open parents: #970 bae72deee7ef90a5e27618c23424c27293c88b72,
#968 cbc819b6b862f6ec14ecb30b8659819e800fce76,
#969 acbcfa898f011f976632625194410d5fae8cccd5.
Fetched upstream main: 5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3.
All are ancestors of the integration base; #970 already includes #968.
The separate integration merge had no conflicts. Original worktrees and fixtures
were preserved. This new main-based PR includes inherited parent commits.
