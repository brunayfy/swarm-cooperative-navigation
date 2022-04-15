// Using d3 svg representations: https://github.com/d3/d3/blob/main/API.md
let scaleFactor = 1;
const dotbotcolors = d3.scaleOrdinal(d3.schemeCategory10).range();

const playbuttonMinX = 175;
const playbuttonMaxX = 285;
const playbuttonMinSpeed = 1.00;
const playbuttonMaxSpeed = 10.00;

function coordinates2pixels(x, y) {
    return 10 * x, 10 * y;
}

const gettingThingsInPlace = () => {
    // arming click events and tooltips on buttons
    $("#pausebutton")
        .click(() => {
            $.post('pause')
        })
        .attr('title', 'Pause the simulation.');
    $("#frameforwardbutton")
        .click(() => {
            $.post('frameforward')
        })
        .attr('title', 'Advance the simulation by one event.');
    $("#playbuttonsliderdiv").hover(handlerPlaybuttonsliderdivHoverIn, handlerPlaybuttonsliderdivHoverOut);
    $("#playbuttonslider").hover(handlerPlaybuttonsliderdivHoverIn, handlerPlaybuttonsliderdivHoverOut);
    $("#playbutton")
        .mousedown(slideHandlerMouseDown)
        .attr('title', 'Drag to set the play speed.')
        .hover(handlerPlaybuttonsliderdivHoverIn, handlerPlaybuttonsliderdivHoverOut);
    $("#fastforwardbutton")
        .click(() => {
            $.post('fastforward')
        })
        .attr('title', 'Simulate as fast as possible.');
};

const getFloorplan = () => {
    $.getJSON("/floorplan.json", floorplan => {
        drawFloorplan(floorplan);
    });
};

function drawFloorplan(floorplan) {
    const svg = d3.select("#floorplan");

    // determine scale factor such that map fill entire width of screen
    scaleFactor = ($('body').innerWidth() - 5) / floorplan.width;

    // scale map to fill up screen
    svg.attr("width", scaleFactor * floorplan.width)
        .attr("height", scaleFactor * floorplan.height);

    // position walls
    svg.selectAll("rect")
        .data(floorplan.obstacles)
        .enter().append("rect")
        .attr("x", (d) => scaleFactor * d.x)
        .attr("y", (d) => scaleFactor * d.y)
        .attr("width", (d) => scaleFactor * d.width)
        .attr("height", (d) => scaleFactor * d.height)
        .attr("class", "obstacle");

    // position buttons and labels
    $("#pagetitle").width(scaleFactor * floorplan.width);
    $("#frameforwardbutton").offset({top: scaleFactor * floorplan.height + 70});
    $("#fastforwardbutton").offset({top: scaleFactor * floorplan.height + 70});
    $("#playbuttonsliderdiv").offset({top: scaleFactor * floorplan.height + 70});
    $("#playbuttonslider").offset({top: scaleFactor * floorplan.height + 94});
    $("#playbuttontooltip").offset({top: scaleFactor * floorplan.height + 45});
    $("#playbutton").offset({top: scaleFactor * floorplan.height + 70});
    $("#pausebutton").offset({top: scaleFactor * floorplan.height + 70});
    $("#timelabel").offset({top: scaleFactor * floorplan.height + 70});
    $("#versionlabel").offset({
        top: scaleFactor * floorplan.height + 70, left: scaleFactor * floorplan.width - 190,
    });
}

const getDotBots = () => {
    $.getJSON("/dotbots.json", data => {
        drawDotBots(data);
    });
};

function drawDotBots(data) {
    var svg = d3.select("#floorplan");

    // mode
    $("#pausebutton").css({opacity: data.mode === 'pause' ? 1.00 : 0.50});
    $("#frameforwardbutton").css({opacity: data.mode === 'frameforward' ? 1.00 : 0.50});
    $("#playbutton").css({opacity: data.mode === 'play' ? 1.00 : 0.50});
    $("#fastforwardbutton").css({opacity: data.mode === 'fastforward' ? 1.00 : 0.50});

    // time label
    $("#timelabel").html(data.simulatedTime);

    // position error
    var positionerror = svg.selectAll(".positionerror")
        .data(data.dotbots);
    positionerror
        .attr("x1", (d) => scaleFactor * d.x)
        .attr("y1", (d) => scaleFactor * d.y)
        .attr("x2", (d) => scaleFactor * d.orchestratorview_x)
        .attr("y2", (d) => scaleFactor * d.orchestratorview_y);
    positionerror
        .enter().append("line")
        .attr("x1", (d) => scaleFactor * d.x)
        .attr("y1", (d) => scaleFactor * d.y)
        .attr("x2", (d) => scaleFactor * d.orchestratorview_x)
        .attr("y2", (d) => scaleFactor * d.orchestratorview_y)
        .attr("class", "positionerror");

    // orchestratorview
    var orchestratorview = svg.selectAll(".orchestratorview")
        .data(data.dotbots);
    orchestratorview
        .transition()
        .attr("cx", (d) => scaleFactor * d.orchestratorview_x)
        .attr("cy", (d) => scaleFactor * d.orchestratorview_y);
    //todo: Why there is this orchestratorview red circle?
    orchestratorview
        .enter().append("circle")
        .attr("cx", (d) => scaleFactor * d.orchestratorview_x)
        .attr("cy", (d) => scaleFactor * d.orchestratorview_y)
        .attr("class", "orchestratorview")
        .attr("r", 6);

    // collision path
    var collisionpaths = svg.selectAll(".collisionpath")
        .data(data.dotbots);
    collisionpaths
        .attr("x1", (d) => scaleFactor * d.x)
        .attr("y1", (d) => scaleFactor * d.y)
        .attr("x2", (d) => d.next_bump_x === null ? scaleFactor * d.x : scaleFactor * d.next_bump_x)
        .attr("y2", (d) => d.next_bump_y === null ? scaleFactor * d.y : scaleFactor * d.next_bump_y);
    collisionpaths
        .enter().append("line")
        .attr("x1", (d) => scaleFactor * d.x)
        .attr("y1", (d) => scaleFactor * d.y)
        .attr("x2", (d) => d.next_bump_x === null ? scaleFactor * d.x : scaleFactor * d.next_bump_x)
        .attr("y2", (d) => d.next_bump_y === null ? scaleFactor * d.y : scaleFactor * d.next_bump_y)
        .attr("class", "collisionpath")
        .attr("stroke", (d, i) => dotbotcolors[i % 10]);

    // dotbots
    var dotbots = svg.selectAll(".dotbot")
        .data(data.dotbots);
    dotbots
        .transition()
        .attr("cx", (d) => scaleFactor * d.x)
        .attr("cy", (d) => scaleFactor * d.y);
    dotbots
        .enter().append("circle")
        .attr("cx", (d) => scaleFactor * d.x)
        .attr("cy", (d) => scaleFactor * d.y)
        .attr("fill", "#00ffff")
        .attr("r", data.robotRadius);
    dotbots
        .enter().append("circle")
        .attr("cx", (d) => scaleFactor * d.x)
        .attr("cy", (d) => scaleFactor * d.y)
        .attr("class", "dotbot")
        .attr("fill", (d, i) => dotbotcolors[i % 10])
        .attr("r", 10);

    // disco map complete
    if (data.discomap.complete === true) {
        $(".discomapline").css({stroke: 'green'});
    } else {
        $(".discomapline").css({stroke: 'red'});
    }

    // disco map dots
    var discomapdots = svg.selectAll(".discomapdot")
        .data(data.discomap.dots);
    discomapdots
        .attr("cx", (d) => scaleFactor * d[0])
        .attr("cy", (d) => scaleFactor * d[1]);
    discomapdots
        .enter().append("circle")
        .attr("cx", (d) => scaleFactor * d[0])
        .attr("cy", (d) => scaleFactor * d[1])
        .attr("class", "discomapdot")
        .attr("r", 2);
    discomapdots
        .exit()
        .remove();

    // disco map lines
    var discomaplines = svg.selectAll(".discomapline")
        .data(data.discomap.lines);
    discomaplines
        .attr("x1", (d) => scaleFactor * d[0])
        .attr("y1", (d) => scaleFactor * d[1])
        .attr("x2", (d) => scaleFactor * d[2])
        .attr("y2", (d) => scaleFactor * d[3]);
    discomaplines
        .enter().append("line")
        .attr("x1", (d) => scaleFactor * d[0])
        .attr("y1", (d) => scaleFactor * d[1])
        .attr("x2", (d) => scaleFactor * d[2])
        .attr("y2", (d) => scaleFactor * d[3])
        .attr("class", "discomapline");
    discomaplines
        .exit()
        .remove();
}

function handlerPlaybuttonsliderdivHoverIn(e) {
    $("#playbuttonslider").css({opacity: 0.30});
}

function handlerPlaybuttonsliderdivHoverOut(e) {
    $("#playbuttonslider").css({opacity: 0.10});
}

function slideHandlerMouseDown(e) {

    // avoid default event
    e = e || window.event;

    e.preventDefault();

    // arm handlers
    document.onmousemove = slideHandlerMouseMove;
    document.onmouseup = slideHandlerMouseUp;
}

function slideHandlerMouseMove(e) {

    // avoid default event
    e = e || window.event;
    e.preventDefault();

    // record mouse position
    newX = e.clientX;

    // move play button
    if (newX < playbuttonMinX) {
        newX = playbuttonMinX
    }

    if (newX > playbuttonMaxX) {
        newX = playbuttonMaxX
    }

    $("#playbutton").offset({left: newX - 25});

    // compute current speed setting
    portion = (newX - playbuttonMinX) / (playbuttonMaxX - playbuttonMinX)
    speed = Math.round(playbuttonMinSpeed + portion * (playbuttonMaxSpeed - playbuttonMinSpeed));

    // display tooltip
    $("#playbuttontooltip").offset({left: newX - 20}).html(speed + ' x');
}

function slideHandlerMouseUp(e) {

    // avoid default event
    e = e || window.event;
    e.preventDefault();

    // disarm handlers
    document.onmousemove = null;
    document.onmouseup = null;

    // remove tooltip
    $("#playbuttontooltip").html('').offset({left: -100});


    // determine speed
    let buttonX = $("#playbutton").position().left + 25;
    let portion = (buttonX - playbuttonMinX) / (playbuttonMaxX - playbuttonMinX)
    let speed = playbuttonMinSpeed + portion * (playbuttonMaxSpeed - playbuttonMinSpeed);

    // send command
    $.ajax({
        type: "POST", url: 'play', contentType: 'application/json', data: JSON.stringify({
            'speed': speed,
        })
    });
}
