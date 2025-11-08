-- pfPolygon: Polygon zone rendering for quest objectives
-- Creates retail-like objective zones around mob spawn points

pfPolygon = CreateFrame("Frame", "pfPolygon", UIParent)
pfPolygon.zones = {}        -- Stores computed polygon zones per quest
pfPolygon.textures = {}     -- Texture pool for rendering
pfPolygon.texturepool = {}  -- Available textures for reuse
pfPolygon.maxpool = 600     -- Maximum textures in pool (optimized for triangle fill)

-- Configuration (will be overridden by pfQuestConfig)
pfPolygon.config = {
  enabled = 1,
  epsilon = 15,            -- Clustering distance in yards (lower = tighter, more zones)
  fillopacity = 0.3,       -- Fill transparency
  borderopacity = 0.7,     -- Border transparency
  maxvertices = 20,        -- Maximum vertices per polygon
  fillresolution = 8,      -- Size of fill textures in pixels (larger = faster but less smooth)
  trianglefill = 1,        -- Use triangle-based fill (1) or grid-based fill (0)
  padding = 2,             -- Padding around zones in yards (expands polygon outward)
}

-- ===================================================================
-- HELPER FUNCTIONS
-- ===================================================================

-- Calculate Euclidean distance between two points
local function Distance(x1, y1, x2, y2)
  local dx = x2 - x1
  local dy = y2 - y1
  return math.sqrt(dx * dx + dy * dy)
end

-- Cross product for three points (o, a, b)
-- Returns: > 0 for counter-clockwise turn, < 0 for clockwise, 0 for collinear
local function CrossProduct(o, a, b)
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
end

-- Sort points by x-coordinate, then by y-coordinate
local function SortPoints(points)
  table.sort(points, function(a, b)
    if a.x == b.x then
      return a.y < b.y
    end
    return a.x < b.x
  end)
end

-- ===================================================================
-- CLUSTERING ALGORITHM (DBSCAN-style)
-- ===================================================================

-- Cluster spawn points into groups based on distance threshold
-- Points within epsilon yards are grouped together
function pfPolygon:ClusterSpawns(coords, epsilon)
  if not coords or table.getn(coords) < 3 then
    return nil  -- Need at least 3 points for a polygon
  end

  epsilon = epsilon or self.config.epsilon
  local minPts = 2  -- Minimum neighbors required to be a "core" point (prevents sparse bridges)
  local clusters = {}
  local visited = {}
  local clustered = {}

  -- Helper: Find all neighbors within epsilon distance
  local function GetNeighbors(pointIdx)
    local neighbors = {}
    local px, py = coords[pointIdx][1], coords[pointIdx][2]

    for i = 1, table.getn(coords) do
      if i ~= pointIdx then
        local dist = Distance(px, py, coords[i][1], coords[i][2])
        if dist <= epsilon then
          table.insert(neighbors, i)
        end
      end
    end

    return neighbors
  end

  -- Helper: Expand cluster from seed point
  local function ExpandCluster(pointIdx, neighbors, clusterIdx)
    table.insert(clusters[clusterIdx], pointIdx)
    clustered[pointIdx] = true

    local i = 1
    while i <= table.getn(neighbors) do
      local neighborIdx = neighbors[i]

      if not visited[neighborIdx] then
        visited[neighborIdx] = true
        local neighborNeighbors = GetNeighbors(neighborIdx)

        -- Only expand through dense points (with minPts neighbors)
        -- This prevents sparse "bridge" points from connecting separate clusters
        if table.getn(neighborNeighbors) >= minPts then
          for _, nn in ipairs(neighborNeighbors) do
            table.insert(neighbors, nn)
          end
        end
      end

      if not clustered[neighborIdx] then
        table.insert(clusters[clusterIdx], neighborIdx)
        clustered[neighborIdx] = true
      end

      i = i + 1
    end
  end

  -- Main DBSCAN loop
  for i = 1, table.getn(coords) do
    if not visited[i] then
      visited[i] = true
      local neighbors = GetNeighbors(i)

      -- Start new cluster only if point is "dense" (has minPts neighbors)
      if table.getn(neighbors) >= minPts then
        local clusterIdx = table.getn(clusters) + 1
        clusters[clusterIdx] = {}
        ExpandCluster(i, neighbors, clusterIdx)
      end
      -- Sparse/isolated points are now excluded (no single-point clusters)
    end
  end

  -- Convert cluster indices to actual coordinate arrays
  local result = {}
  for _, cluster in ipairs(clusters) do
    if table.getn(cluster) >= 3 then  -- Only return clusters with 3+ points
      local clusterCoords = {}
      for _, idx in ipairs(cluster) do
        table.insert(clusterCoords, {
          x = coords[idx][1],
          y = coords[idx][2]
        })
      end
      table.insert(result, clusterCoords)
    end
  end

  return result
end

-- ===================================================================
-- CONVEX HULL ALGORITHM (Andrew's Monotone Chain)
-- ===================================================================

-- Compute convex hull using Andrew's Monotone Chain algorithm
-- Returns array of vertices forming the convex hull polygon
function pfPolygon:ConvexHull(points)
  if not points or table.getn(points) < 3 then
    return nil
  end

  -- Create copy to avoid modifying original
  local pts = {}
  for i, p in ipairs(points) do
    pts[i] = {x = p.x, y = p.y}
  end

  -- Sort points by x, then y
  SortPoints(pts)

  -- Build lower hull
  local lower = {}
  for _, p in ipairs(pts) do
    local lowerlen = table.getn(lower)
    while lowerlen >= 2 and CrossProduct(lower[lowerlen - 1], lower[lowerlen], p) <= 0 do
      table.remove(lower)
      lowerlen = table.getn(lower)
    end
    table.insert(lower, p)
  end

  -- Build upper hull
  local upper = {}
  for i = table.getn(pts), 1, -1 do
    local p = pts[i]
    local upperlen = table.getn(upper)
    while upperlen >= 2 and CrossProduct(upper[upperlen - 1], upper[upperlen], p) <= 0 do
      table.remove(upper)
      upperlen = table.getn(upper)
    end
    table.insert(upper, p)
  end

  -- Remove last point of each half because it's repeated
  table.remove(lower)
  table.remove(upper)

  -- Concatenate lower and upper hull
  for _, p in ipairs(upper) do
    table.insert(lower, p)
  end

  return lower
end

-- Expand polygon outward by padding amount
-- Moves each vertex away from centroid
function pfPolygon:ExpandPolygon(vertices, padding)
  if not vertices or table.getn(vertices) < 3 or not padding or padding <= 0 then
    return vertices
  end

  -- Calculate centroid
  local cx, cy = 0, 0
  local count = table.getn(vertices)

  for _, v in ipairs(vertices) do
    cx = cx + v.x
    cy = cy + v.y
  end

  cx = cx / count
  cy = cy / count

  -- Expand each vertex outward from centroid
  local expanded = {}

  for _, v in ipairs(vertices) do
    -- Vector from centroid to vertex
    local dx = v.x - cx
    local dy = v.y - cy
    local dist = math.sqrt(dx * dx + dy * dy)

    if dist > 0 then
      -- Normalize and scale by padding
      local nx = dx / dist
      local ny = dy / dist

      -- Move vertex outward
      table.insert(expanded, {
        x = v.x + nx * padding,
        y = v.y + ny * padding
      })
    else
      -- Vertex is at centroid (shouldn't happen), keep as-is
      table.insert(expanded, {x = v.x, y = v.y})
    end
  end

  return expanded
end

-- ===================================================================
-- POLYGON SIMPLIFICATION (Douglas-Peucker)
-- ===================================================================

-- Simplify polygon if it has too many vertices
-- Uses Douglas-Peucker algorithm
function pfPolygon:SimplifyPolygon(vertices, tolerance)
  if not vertices or table.getn(vertices) <= self.config.maxvertices then
    return vertices
  end

  tolerance = tolerance or 5  -- Default tolerance in yards

  -- Helper: Perpendicular distance from point to line segment
  local function PerpendicularDistance(point, lineStart, lineEnd)
    local dx = lineEnd.x - lineStart.x
    local dy = lineEnd.y - lineStart.y

    if dx == 0 and dy == 0 then
      return Distance(point.x, point.y, lineStart.x, lineStart.y)
    end

    local t = ((point.x - lineStart.x) * dx + (point.y - lineStart.y) * dy) / (dx * dx + dy * dy)
    t = math.max(0, math.min(1, t))

    local projX = lineStart.x + t * dx
    local projY = lineStart.y + t * dy

    return Distance(point.x, point.y, projX, projY)
  end

  -- Recursive Douglas-Peucker
  local function DouglasPeucker(points, startIdx, endIdx, tolerance)
    local maxDist = 0
    local maxIdx = 0

    for i = startIdx + 1, endIdx - 1 do
      local dist = PerpendicularDistance(points[i], points[startIdx], points[endIdx])
      if dist > maxDist then
        maxDist = dist
        maxIdx = i
      end
    end

    local result = {}

    if maxDist > tolerance then
      -- Recursively simplify
      local left = DouglasPeucker(points, startIdx, maxIdx, tolerance)
      local right = DouglasPeucker(points, maxIdx, endIdx, tolerance)

      -- Combine results
      for _, p in ipairs(left) do
        table.insert(result, p)
      end
      for i = 2, table.getn(right) do
        table.insert(result, right[i])
      end
    else
      -- Keep only endpoints
      table.insert(result, points[startIdx])
      table.insert(result, points[endIdx])
    end

    return result
  end

  if table.getn(vertices) < 3 then
    return vertices
  end

  local simplified = DouglasPeucker(vertices, 1, table.getn(vertices), tolerance)

  -- If still too many vertices, increase tolerance and try again
  if table.getn(simplified) > self.config.maxvertices then
    return self:SimplifyPolygon(vertices, tolerance * 1.5)
  end

  return simplified
end

-- ===================================================================
-- POLYGON TRIANGULATION
-- ===================================================================

-- Decompose convex polygon into triangles using fan triangulation
-- For a convex polygon with n vertices, creates (n-2) triangles
function pfPolygon:TriangulateFan(vertices)
  if not vertices or table.getn(vertices) < 3 then
    return nil
  end

  local triangles = {}
  local anchor = vertices[1]
  local numVertices = table.getn(vertices)

  -- Create fan from first vertex to all other edges
  for i = 2, numVertices - 1 do
    table.insert(triangles, {
      anchor,
      vertices[i],
      vertices[i + 1]
    })
  end

  return triangles
end

-- ===================================================================
-- POLYGON ZONE COMPUTATION
-- ===================================================================

-- Compute polygon zones for a set of spawn coordinates
-- Returns array of polygon zones (each zone is an array of vertices)
function pfPolygon:ComputeZones(coords)
  if not coords or table.getn(coords) < 3 then
    return nil
  end

  -- Cluster spawn points into groups
  local clusters = self:ClusterSpawns(coords, self.config.epsilon)
  if not clusters then
    return nil
  end

  -- Compute convex hull for each cluster
  local zones = {}
  for _, cluster in ipairs(clusters) do
    local hull = self:ConvexHull(cluster)
    if hull and table.getn(hull) >= 3 then
      -- Expand polygon outward by padding
      hull = self:ExpandPolygon(hull, self.config.padding)

      -- Simplify if needed
      hull = self:SimplifyPolygon(hull)
      table.insert(zones, hull)
    end
  end

  return table.getn(zones) > 0 and zones or nil
end

-- ===================================================================
-- ZONE STORAGE AND MANAGEMENT
-- ===================================================================

-- Store polygon zones for a quest objective
function pfPolygon:AddZones(questid, objectiveid, coords, mapid, color)
  if not self.config.enabled or self.config.enabled == 0 then
    return
  end

  local zones = self:ComputeZones(coords)
  if not zones then
    return
  end

  -- Initialize storage
  self.zones[questid] = self.zones[questid] or {}
  self.zones[questid][objectiveid] = {
    zones = zones,
    mapid = mapid,
    color = color or {r = 1, g = 1, b = 0}  -- Default yellow
  }
end

-- Clear zones for a specific quest
function pfPolygon:ClearQuest(questid)
  if self.zones[questid] then
    self.zones[questid] = nil
  end
end

-- Clear all zones
function pfPolygon:ClearAll()
  self.zones = {}
end

-- ===================================================================
-- TEXTURE POOL MANAGEMENT
-- ===================================================================

-- Get a texture from the pool or create a new one
function pfPolygon:GetTexture(parent)
  local tex

  if table.getn(self.texturepool) > 0 then
    tex = table.remove(self.texturepool)
    tex:SetParent(parent)
  else
    tex = parent:CreateTexture(nil, "ARTWORK")
  end

  table.insert(self.textures, tex)
  return tex
end

-- Release a texture back to the pool
function pfPolygon:ReleaseTexture(tex)
  if not tex then return end

  tex:Hide()
  tex:ClearAllPoints()
  tex:SetTexture(nil)

  -- Add to pool if under limit
  if table.getn(self.texturepool) < self.maxpool then
    table.insert(self.texturepool, tex)
  end
end

-- Release all textures
function pfPolygon:ReleaseAllTextures()
  for _, tex in ipairs(self.textures) do
    self:ReleaseTexture(tex)
  end
  self.textures = {}
end

-- ===================================================================
-- COORDINATE TRANSFORMATION
-- ===================================================================

-- Transform world coordinates (0-100) to map pixel coordinates
-- Based on pfQuest's map.lua coordinate system
function pfPolygon:TransformCoords(x, y)
  if not WorldMapButton then
    return nil, nil
  end

  local px = x / 100 * WorldMapButton:GetWidth()
  local py = y / 100 * WorldMapButton:GetHeight()

  return px, py
end

-- ===================================================================
-- POLYGON RENDERING - BORDER
-- ===================================================================

-- Draw a line between two points
-- Adapted from pfQuest's route.lua
local function DrawLine(parent, x1, y1, x2, y2, color, alpha, textures)
  local dx = x2 - x1
  local dy = y2 - y1
  local distance = math.sqrt(dx * dx + dy * dy)

  -- Calculate number of dots based on distance
  local dots = math.ceil(distance / 3)  -- 3 pixels between dots

  if dots < 2 then return end

  for i = 0, dots do
    local t = i / dots
    local x = x1 + dx * t
    local y = y1 + dy * t

    local tex = pfPolygon:GetTexture(parent)
    tex:SetWidth(3)
    tex:SetHeight(3)
    tex:SetTexture(1, 1, 1)  -- White texture
    tex:SetVertexColor(color.r, color.g, color.b)
    tex:SetAlpha(alpha)
    tex:SetPoint("CENTER", parent, "TOPLEFT", x, -y)
    tex:Show()

    table.insert(textures, tex)
  end
end

-- Draw polygon border by connecting vertices
function pfPolygon:DrawBorder(vertices, parent, color, alpha)
  if not vertices or table.getn(vertices) < 3 then
    return
  end

  local textures = {}
  local numVertices = table.getn(vertices)

  -- Draw lines connecting each vertex
  for i = 1, numVertices do
    local v1 = vertices[i]
    local nextIdx = math.mod(i, numVertices) + 1  -- Wrap around to first vertex
    local v2 = vertices[nextIdx]

    local x1, y1 = self:TransformCoords(v1.x, v1.y)
    local x2, y2 = self:TransformCoords(v2.x, v2.y)

    if x1 and y1 and x2 and y2 then
      DrawLine(parent, x1, y1, x2, y2, color, alpha, textures)
    end
  end

  return textures
end

-- ===================================================================
-- POLYGON RENDERING - FILL
-- ===================================================================

-- Point-in-polygon test using ray casting algorithm
local function PointInPolygon(x, y, vertices)
  local inside = false
  local j = table.getn(vertices)

  for i = 1, table.getn(vertices) do
    local vi = vertices[i]
    local vj = vertices[j]

    if ((vi.y > y) ~= (vj.y > y)) and
       (x < (vj.x - vi.x) * (y - vi.y) / (vj.y - vi.y) + vi.x) then
      inside = not inside
    end

    j = i
  end

  return inside
end

-- Get bounding box of polygon
local function GetBoundingBox(vertices)
  local minX, minY = 99999, 99999
  local maxX, maxY = -99999, -99999

  for _, v in ipairs(vertices) do
    minX = math.min(minX, v.x)
    minY = math.min(minY, v.y)
    maxX = math.max(maxX, v.x)
    maxY = math.max(maxY, v.y)
  end

  return minX, minY, maxX, maxY
end

-- Fill a single triangle using scanline rasterization
function pfPolygon:FillTriangle(v1, v2, v3, parent, color, alpha)
  local textures = {}
  local resolution = self.config.fillresolution

  -- Sort vertices by y coordinate (top to bottom)
  local verts = {v1, v2, v3}
  table.sort(verts, function(a, b) return a.y < b.y end)
  local top = verts[1]
  local mid = verts[2]
  local bottom = verts[3]

  -- Skip degenerate triangles
  if top.y == bottom.y then
    return textures
  end

  -- Transform to screen coordinates
  local function toScreen(v)
    local px, py = self:TransformCoords(v.x, v.y)
    if not px or not py then return nil end
    return {x = px, y = py}
  end

  local topS = toScreen(top)
  local midS = toScreen(mid)
  local bottomS = toScreen(bottom)

  if not topS or not midS or not bottomS then
    return textures
  end

  -- Helper: linear interpolation of X given Y on edge from v1 to v2
  local function interpX(y, v1S, v2S)
    if v2S.y == v1S.y then
      return v1S.x
    end
    local t = (y - v1S.y) / (v2S.y - v1S.y)
    return v1S.x + t * (v2S.x - v1S.x)
  end

  -- Scanline fill: iterate through each horizontal line
  local minY = math.floor(topS.y)
  local maxY = math.ceil(bottomS.y)

  for y = minY, maxY, resolution do
    local leftX, rightX

    -- Determine which edges to use based on current Y
    if y <= midS.y then
      -- Upper part: from top to mid and top to bottom
      leftX = interpX(y, topS, midS)
      rightX = interpX(y, topS, bottomS)
    else
      -- Lower part: from mid to bottom and top to bottom
      leftX = interpX(y, midS, bottomS)
      rightX = interpX(y, topS, bottomS)
    end

    -- Ensure leftX < rightX
    if leftX > rightX then
      leftX, rightX = rightX, leftX
    end

    local width = rightX - leftX

    -- Only create texture if width is significant
    if width > 1 then
      local tex = self:GetTexture(parent)
      tex:SetWidth(math.max(1, width))
      tex:SetHeight(resolution)
      tex:SetTexture(1, 1, 1)
      tex:SetVertexColor(color.r, color.g, color.b)
      tex:SetAlpha(alpha)
      tex:SetPoint("CENTER", parent, "TOPLEFT", (leftX + rightX) / 2, -y)
      tex:Show()

      table.insert(textures, tex)

      -- Safety limit
      if table.getn(textures) > 50 then
        return textures
      end
    end
  end

  return textures
end

-- Fill polygon with semi-transparent textures (OLD GRID METHOD - kept as fallback)
function pfPolygon:FillPolygonGrid(vertices, parent, color, alpha)
  if not vertices or table.getn(vertices) < 3 then
    return
  end

  local textures = {}
  local resolution = self.config.fillresolution

  -- Get bounding box
  local minX, minY, maxX, maxY = GetBoundingBox(vertices)

  -- Transform to screen coordinates for proper spacing
  local minPx, minPy = self:TransformCoords(minX, minY)
  local maxPx, maxPy = self:TransformCoords(maxX, maxY)

  if not minPx or not maxPx then
    return textures
  end

  -- Sample grid points and fill if inside polygon
  local stepX = resolution * 100 / WorldMapButton:GetWidth()  -- Convert pixels back to 0-100 coords
  local stepY = resolution * 100 / WorldMapButton:GetHeight()

  local y = minY
  while y <= maxY do
    local x = minX
    while x <= maxX do
      if PointInPolygon(x, y, vertices) then
        local px, py = self:TransformCoords(x, y)

        if px and py then
          local tex = self:GetTexture(parent)
          tex:SetWidth(resolution)
          tex:SetHeight(resolution)
          tex:SetTexture(1, 1, 1)  -- White texture
          tex:SetVertexColor(color.r, color.g, color.b)
          tex:SetAlpha(alpha)
          tex:SetPoint("CENTER", parent, "TOPLEFT", px, -py)
          tex:Show()

          table.insert(textures, tex)

          -- Safety limit: don't create too many textures
          if table.getn(textures) > 200 then
            return textures
          end
        end
      end

      x = x + stepX
    end
    y = y + stepY
  end

  return textures
end

-- Fill polygon with semi-transparent textures (NEW TRIANGLE METHOD)
function pfPolygon:FillPolygon(vertices, parent, color, alpha)
  -- Check config for which method to use
  if self.config.trianglefill == 0 then
    -- Use old grid method as fallback
    return self:FillPolygonGrid(vertices, parent, color, alpha)
  end

  -- Use new triangle-based fill
  if not vertices or table.getn(vertices) < 3 then
    return {}
  end

  -- Decompose polygon into triangles
  local triangles = self:TriangulateFan(vertices)
  if not triangles then
    return {}
  end

  local textures = {}

  -- Fill each triangle
  for _, triangle in ipairs(triangles) do
    local triTextures = self:FillTriangle(triangle[1], triangle[2], triangle[3], parent, color, alpha)
    if triTextures then
      for _, tex in ipairs(triTextures) do
        table.insert(textures, tex)
      end
    end

    -- Safety limit across all triangles
    if table.getn(textures) > 200 then
      return textures
    end
  end

  return textures
end

-- ===================================================================
-- MAIN RENDERING FUNCTIONS
-- ===================================================================

-- Render all polygon zones on the current map
function pfPolygon:RenderZones()
  if not self.config.enabled or self.config.enabled == 0 then
    return
  end

  -- Check if WorldMapButton exists
  if not WorldMapButton then
    return
  end

  -- Clear previous rendering
  self:ReleaseAllTextures()

  -- Get current map ID using pfMap's method
  local currentMap = nil
  if pfMap and pfMap.GetMapID then
    currentMap = pfMap:GetMapID(GetCurrentMapContinent(), GetCurrentMapZone())
  end

  if not currentMap then
    return
  end

  -- Render zones for all active quests
  for questid, objectives in pairs(self.zones) do
    for objectiveid, data in pairs(objectives) do
      -- Only render zones for current map
      if data.mapid == currentMap or not data.mapid then
        for _, vertices in ipairs(data.zones) do
          -- Draw fill
          self:FillPolygon(vertices, WorldMapButton, data.color, self.config.fillopacity)

          -- Draw border
          self:DrawBorder(vertices, WorldMapButton, data.color, self.config.borderopacity)
        end
      end
    end
  end
end

-- Clear all rendered zones
function pfPolygon:ClearRenderedZones()
  self:ReleaseAllTextures()
end

-- ===================================================================
-- INITIALIZATION
-- ===================================================================

-- Initialize polygon system
pfPolygon:RegisterEvent("PLAYER_LOGIN")
pfPolygon:SetScript("OnEvent", function()
  -- Load config from pfQuest_config if available
  if pfQuest_config then
    if pfQuest_config.polygonzones == "1" then
      pfPolygon.config.enabled = 1
    elseif pfQuest_config.polygonzones == "0" then
      pfPolygon.config.enabled = 0
    end
  end
end)
