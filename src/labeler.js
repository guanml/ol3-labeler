(function() {

  /**
   * Automatic label placement for ol3.
   */
  OL3Labeler = function(options) {

    this.labels = options.labels;
    this.bounds = options.bounds;
    this.anchors = options.anchors || [];
    this.features =  options.features || [];

    // use the lower left of the label bbox as
    // anchor if undefined
    // TODO
    if (!this.anchors.length) {
      for (var i = 0, len = this.labels.length; i < len; i++) {
        var c = this.labels[i].extent.slice(0, 2);
        this.anchors.push(new ol.geom.Circle(c, 2)); // TODO
      }
    }

    this.labeler = {};

    this.max_move = 5;
    this.max_angle = 0.5;
    this.rej = 0;

    // weights
    this.w_len = 0.2; // leader line length
    this.w_inter = 1.0; // leader line intersection
    this.w_lab2 = 30.0; // label-label overlap
    this.w_lab_anc = 30.0; // label-anchor overlap
    this.w_orient = 3.0; // orientation bias
  };

  /**
   * Energy function, tailored for label placement
   */
  OL3Labeler.prototype.getEnergy = function(index) {

    var ener = 0;
    var anchorPos = this.anchors[index].getCenter();
    var dx = this.labels[index].extent[0] - anchorPos[0];
    var dy = anchorPos[1] - this.labels[index].extent[1];
    var dist = Math.sqrt(dx * dx + dy * dy);

    // penalty for anchor distance
    ener += dist * this.w_len;

    var ext1 = this.labels[index].extent;

    for (var i = 0, ii = this.labels.length; i < ii; i++) {
      if (i != index) {

        // penalty for label-label overlap
        var ext2 = this.labels[i].extent;

        if (ol.extent.intersects(ext1, ext2)) {
          var overlap_area = ol.extent.getIntersectionArea(ext1, ext2);
          ener += overlap_area * this.w_lab2;
        }

        // penalty for label-anchor overlap
        if (this.anchors[i].intersectsExtent(ext1)) {
          var ext = this.anchors[i].getExtent();
          var overlap_area = ol.extent.getIntersectionArea(ext, ext2);
          ener += overlap_area * this.w_lab_anc;
        }
      }
    }

    // penalty for label-feature-overlap
    // TODO
    if (this.features.length) {
      for (var j = 0; j < this.features.length; j++) {
        var geom = this.features[j].getGeometry();
        var area = ol.extent.getIntersectionArea(ext1, geom.getExtent());

        if (area /* fast check */ &&
            this.features[j].getGeometry().intersectsExtent(ext1)) {

          var overlap_area = ol.extent.getIntersectionArea(
          this.features[j].getGeometry().getExtent(), 1);
          ener += overlap_area * this.w_lab_anc;
        }
      }
    }

    return ener;
  };

  /**
   * Monte Carlo translation move
   */
  OL3Labeler.prototype.mcmove = function(currT) {

    // select a random label
    var i = Math.floor(Math.random() * this.labels.length);

    // save old coordinates
    var x_old = this.labels[i].extent[0];
    var y_old = this.labels[i].extent[1];

    var old_energy = this.getEnergy(i);

    // random translation
    var moveX = (Math.random() - 0.5) * this.max_move;
    var moveY = (Math.random() - 0.5) * this.max_move;

    this.labels[i].extent[0] += moveX;
    this.labels[i].extent[2] += moveX;
    this.labels[i].extent[1] += moveY;
    this.labels[i].extent[3] += moveY;

    // new energy
    var new_energy = this.getEnergy(i);

    // delta E
    var delta_energy = new_energy - old_energy;

    // hard wall boundaries
    var moveBack = this.bounds && !ol.extent.containsExtent(
        this.bounds, this.labels[i].extent);

    if (moveBack || Math.random() > Math.exp(-delta_energy / currT)) {
      // move back to old coordinates
      var moveX = this.labels[i].extent[0] - x_old;
      var moveY = this.labels[i].extent[1] - y_old;
      this.labels[i].extent[0] -= moveX;
      this.labels[i].extent[2] -= moveX;
      this.labels[i].extent[1] -= moveY;
      this.labels[i].extent[3] -= moveY;
    }
  };

  /**
   * Monte Carlo rotation move
   */
  OL3Labeler.prototype.mcrotate = function(currT) {
      // select a random label
      var i = Math.floor(Math.random() * this.labels.length);

      // save old coordinates
      var x_old = this.labels[i].extent[0];
      var y_old = this.labels[i].extent[1];

      // old energy
      var old_energy = this.getEnergy(i);

      // random angle
      var angle = (Math.random() - 0.5) * this.max_angle;

      var s = Math.sin(angle);
      var c = Math.cos(angle);

      // translate label (relative to anchor at origin):
      var anchorPos = this.anchors[i].getCenter();
      this.labels[i].extent[0] -= anchorPos[0];
      this.labels[i].extent[1] -= anchorPos[1];
      this.labels[i].extent[2] -= anchorPos[0];
      this.labels[i].extent[3] -= anchorPos[1];

      // rotate label
      var x_new = this.labels[i].extent[0] * c - this.labels[i].extent[1] * s;
      var y_new = this.labels[i].extent[0] * s + this.labels[i].extent[1] * c;
      var x_width = this.labels[i].extent[2] - this.labels[i].extent[0];
      var x_height = this.labels[i].extent[3] - this.labels[i].extent[1];

      // translate label back
      this.labels[i].extent[0] = x_new + anchorPos[0];
      this.labels[i].extent[1] = y_new + anchorPos[1];
      this.labels[i].extent[2] = x_new + anchorPos[0] + x_width;
      this.labels[i].extent[3] = y_new + anchorPos[1] + x_height;

      // hard wall boundaries
      var moveBack = this.bounds && !ol.extent.containsExtent(
          this.bounds, this.labels[i].extent);

      // new energy
      var new_energy = this.getEnergy(i);

      // delta E
      var delta_energy = new_energy - old_energy;

      if (moveBack || Math.random() > Math.exp(-delta_energy / currT)) {
        var moveX = this.labels[i].extent[0] - x_old;
        var moveY = this.labels[i].extent[1] - y_old;
        this.labels[i].extent[0] -= moveX;
        this.labels[i].extent[2] -= moveX;
        this.labels[i].extent[1] -= moveY;
        this.labels[i].extent[3] -= moveY;
      }
  };

  /**
   * Linear cooling
   */
  OL3Labeler.prototype.cooling_schedule = function(currT, initialT, nsweeps) {
    return (currT - (initialT / nsweeps));
  };

  /**
   * Main simulated annealing function
   */
  OL3Labeler.prototype.start = function(nsweeps) {
      var m = this.labels.length,
          currT = 1.0,
          initialT = 1.0;

      for (var i = 0; i < nsweeps; i++) {
        for (var j = 0; j < m; j++) {
          if (Math.random() < 0.5 && this.anchors) {
            this.mcrotate(currT);
          } else {
            this.mcmove(currT);
          }
        }
        currT = this.cooling_schedule(currT, initialT, nsweeps);
      }
  };
})();
