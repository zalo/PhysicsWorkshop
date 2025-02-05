class Hash {
    constructor(spacing, maxNumObjects) {
        this.spacing = spacing;
        this.tableSize = 2 * maxNumObjects;
        this.cellStart = new Int32Array(this.tableSize + 1);
        this.cellEntries = new Int32Array(maxNumObjects);
        this.queryIds = new Int32Array(maxNumObjects);
        this.querySize = 0;
    }

    hashCoords(xi, yi, zi) {
        var h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);	// fantasy function
        return Math.abs(h) % this.tableSize; 
    }

    intCoord(coord) {
        return Math.floor(coord / this.spacing);
    }

    hashPos(pos, nr) {
        return this.hashCoords(
            this.intCoord(pos[3 * nr]), 
            this.intCoord(pos[3 * nr + 1]),
            this.intCoord(pos[3 * nr + 2]));
    }

    create(pos) {
        var numObjects = Math.min(pos.length / 3, this.cellEntries.length);

        // determine cell sizes

        this.cellStart.fill(0);
        this.cellEntries.fill(0);

        for (var i = 0; i < numObjects; i++) {
            var h = this.hashPos(pos, i);
            this.cellStart[h]++;
        }

        // determine cells starts

        var start = 0;
        for (var i = 0; i < this.tableSize; i++) {
            start += this.cellStart[i];
            this.cellStart[i] = start;
        }
        this.cellStart[this.tableSize] = start;	// guard

        // fill in objects ids

        for (var i = 0; i < numObjects; i++) {
            var h = this.hashPos(pos, i);
            this.cellStart[h]--;
            this.cellEntries[this.cellStart[h]] = i;
        }
    }

    query(pos, nr, maxDist) {
        var x0 = this.intCoord(pos[3 * nr] - maxDist);
        var y0 = this.intCoord(pos[3 * nr + 1] - maxDist);
        var z0 = this.intCoord(pos[3 * nr + 2] - maxDist);

        var x1 = this.intCoord(pos[3 * nr] + maxDist);
        var y1 = this.intCoord(pos[3 * nr + 1] + maxDist);
        var z1 = this.intCoord(pos[3 * nr + 2] + maxDist);

        this.querySize = 0;

        for (var xi = x0; xi <= x1; xi++) {
            for (var yi = y0; yi <= y1; yi++) {
                for (var zi = z0; zi <= z1; zi++) {
                    var h = this.hashCoords(xi, yi, zi);
                    var start = this.cellStart[h];
                    var end = this.cellStart[h + 1];

                    for (var i = start; i < end; i++) {
                        this.queryIds[this.querySize] = this.cellEntries[i];
                        this.querySize++;
                    }
                }
            }
        }
    }
};

export default Hash;
