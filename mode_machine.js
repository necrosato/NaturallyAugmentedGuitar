autowatch=1;
var scales = {'major': 			[1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
			  'harmonic_minor': [1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1],
			  'melodic_minor': 	[1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
              'diminished':     [1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
              'augmented':      [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0] 
             };

function get_mode_count(scale) {
    var mode_count = 0;
    for (var i = 0; i < scales[scale].length; i++) {
        if (scales[scale][i] == 1) {
            mode_count++;
        }
    }
    return mode_count;
}

function get_mode_offset(n, scale) {
	//function returns the offset into scale (0 indexed) for a given mode n of a scale
	var ncount = 0;
    var mode_count = get_mode_count(scale);
    var target = ((n-1) % mode_count)+1;
    var mult = Math.floor((n-1)/mode_count);
    for (var i = 0; i < scales[scale].length; i++) {
		if (scales[scale][i] == 1) {
			ncount++;
		}
		if (ncount == target) {
			return (i + (12*mult));
		}
	}
	return 0;
}

function get_mode(n, root, scale, out) {
    // n: mode number (1 indexed)
    // root: root note of mode
    // scale: key in scales
	var mode = [];
	var offset = get_mode_offset(n, scale);
    var mult = Math.floor(offset/12);
    var new_root = root - (offset % 12); // use this for getting mode AT root
    //var new_root = root; // use this for getting mode FROM root
	for (var i = 0; i < 12; i++) {	
		var index = (i + offset) % 12;
		if (scales[scale][index] == 1) {
			mode.push(new_root+(index<i?12:0)+(mult*12)+index);
		}
	}
	if (out == 1) {
		outlet(0, mode);
	}
	return mode;
}

function get_chord(n, depth, mode, root, scale, out) {
    // n: chord number in relation to start of mode (1 indexed)
    // depth: depth of chord (1 indexed)
    // mode: mode number of scale (1 indexed)
    // root: root note of mode
    // scale: key in scales
	var root_mode = get_mode(mode, root, scale, 0);
    var mcount = root_mode.length;
	var mode_offset = root_mode[(n-1)%mcount] - root_mode[0];
	var chord_scale = get_mode(((mode+n-2)%mcount)+1, root+mode_offset, scale, 0);
	var chord_mult = Math.floor((n - 1) / mcount);
    var chord = [];
	for (var i = 0; i < depth; i++) {
		var index = (i * 2) % mcount;
        var note_mult = Math.floor((i * 2) / mcount);
        if ((i * 2 >= mcount) && (mcount % 2 == 0) && (note_mult % 2 == 1)) {
            // this should only apply to diminished and augmented scales
            index++;
        }
        chord.push(chord_scale[index]+(note_mult*12)+(chord_mult*12)); 
	}
	if (out == 1) {
		outlet(0, chord);
	}
    return chord;
}

function get_arpeggio(n, depth, mode, root, scale, alen, direction, out) {
    // n: arpeggio/chord number in relation to start of mode (1 indexed)
    // depth: depth of pattern
    // mode: mode number of scale (1 indexed)
    // root: root note of mode
    // scale: key in scales
    // alen: number of notes in arpeggio
    // direction: -2 for down, -1 for down/up, 1 for up/down, 2 for up
    // out: output to max (0 or 1)
    var chord = get_chord(n, depth, mode, root, scale, 0);
    var arpeggio = [];
    var compression = ((chord[depth-1] - chord[0]) < 12 ? 12 : 24);
    if (direction == -2) {
        for (var i = 0; i < alen; i++) {
            var index = (depth - (i % depth)) % depth;
            var note_mult = Math.floor((i-1+depth)/depth) * (i>0?-1:0);
            note_mult *= compression;
            arpeggio.push(chord[index]+note_mult);
        }
    }
    if (direction == -1) {
    // TODO: Make this work
        arpeggio = get_arpeggio(n, depth, mode, root, scale, Math.ceil(alen/2), -2, 0);
        var mi = Math.floor(alen/2)-1;
        for (var i = 0; i < Math.floor(alen/2); i++) {
            arpeggio.push(arpeggio[mi-i]);
        }
    }
    if (direction == 1) {
        arpeggio = get_arpeggio(n, depth, mode, root, scale, Math.ceil(alen/2), 2, 0);
        var mi = Math.floor(alen/2)-1;
        for (var i = 0; i < Math.floor(alen/2); i++) {
            arpeggio.push(arpeggio[mi-i]);
        }
    }
    if (direction == 2) {
        for (var i = 0; i < alen; i++) {
            var index = i % depth;
            var note_mult = Math.floor(i/depth);
            note_mult *= compression;
            arpeggio.push(chord[index]+note_mult);
        }
    }
	if (out == 1) {
		outlet(0, arpeggio);
	}
    return arpeggio;
}

function split_list(notes) {
	for (var i = 0; i < notes.length; i++) {
		outlet(0, notes[i]);
	}
}
