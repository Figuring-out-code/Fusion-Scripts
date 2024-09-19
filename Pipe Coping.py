import adsk.core
import adsk.fusion
import traceback
import math

def get_body_transform(body):
    return body.assemblyContext.transform if body.assemblyContext else adsk.core.Matrix3D.create()

def transform_point(point, transform):
    point_clone = point.copy()
    point_clone.transformBy(transform)
    return point_clone

def transform_vector(vector, transform):
    vector_clone = vector.copy()
    vector_clone.transformBy(transform)
    return vector_clone

def is_face_oriented_inwards(face, body):
    try:
        u, v = 0.5, 0.5
        centroid = adsk.core.Point2D.create(u, v)
        result, world_point = face.evaluator.getPointAtParameter(centroid)
        if not result:
            return False

        result, normal = face.evaluator.getNormalAtPoint(world_point)
        if not result:
            return False

        transform = get_body_transform(body)
        world_point = transform_point(world_point, transform)
        normal = transform_vector(normal, transform)

        bbox = body.boundingBox
        body_centroid = adsk.core.Point3D.create(
            (bbox.minPoint.x + bbox.maxPoint.x) / 2,
            (bbox.minPoint.y + bbox.maxPoint.y) / 2,
            (bbox.minPoint.z + bbox.maxPoint.z) / 2
        )
        body_centroid = transform_point(body_centroid, transform)

        direction_to_centroid = adsk.core.Vector3D.create(
            body_centroid.x - world_point.x,
            body_centroid.y - world_point.y,
            body_centroid.z - world_point.z
        )
        direction_to_centroid.normalize()

        normal.normalize()
        dot_product = normal.dotProduct(direction_to_centroid)

        return dot_product > 0
    except Exception:
        return False

def is_face_oriented_outwards(face, body):
    try:
        u, v = 0.5, 0.5
        centroid = adsk.core.Point2D.create(u, v)
        result, world_point = face.evaluator.getPointAtParameter(centroid)
        if not result:
            return False

        result, normal = face.evaluator.getNormalAtPoint(world_point)
        if not result:
            return False

        transform = get_body_transform(body)
        world_point = transform_point(world_point, transform)
        normal = transform_vector(normal, transform)

        bbox = body.boundingBox
        body_centroid = adsk.core.Point3D.create(
            (bbox.minPoint.x + bbox.maxPoint.x) / 2,
            (bbox.minPoint.y + bbox.maxPoint.y) / 2,
            (bbox.minPoint.z + bbox.maxPoint.z) / 2
        )
        body_centroid = transform_point(body_centroid, transform)

        direction_to_centroid = adsk.core.Vector3D.create(
            body_centroid.x - world_point.x,
            body_centroid.y - world_point.y,
            body_centroid.z - world_point.z
        )
        direction_to_centroid.normalize()

        normal.normalize()
        dot_product = normal.dotProduct(direction_to_centroid)

        return dot_product < 0
    except Exception:
        return False

def point_to_str(point):
    return f"({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"

def vector_to_str(vector):
    return f"({vector.x:.3f}, {vector.y:.3f}, {vector.z:.3f})"

def is_center_plane_face(face, body):
    try:
        transform = get_body_transform(body)
        bbox = body.boundingBox
        center_point = adsk.core.Point3D.create(
            (bbox.minPoint.x + bbox.maxPoint.x) / 2,
            (bbox.minPoint.y + bbox.maxPoint.y) / 2,
            (bbox.minPoint.z + bbox.maxPoint.z) / 2
        )
        center_point = transform_point(center_point, transform)
        face_center = adsk.core.Point3D.create(0, 0, 0)

        vertices = face.vertices
        for vertex in vertices:
            vertex_point = transform_point(vertex.geometry, transform)
            face_center.x += vertex_point.x
            face_center.y += vertex_point.y
            face_center.z += vertex_point.z

        face_center.x /= vertices.count
        face_center.y /= vertices.count
        face_center.z /= vertices.count

        tolerance_x = (bbox.maxPoint.x - bbox.minPoint.x) * 0.01
        tolerance_y = (bbox.maxPoint.y - bbox.minPoint.y) * 0.01
        tolerance_z = (bbox.maxPoint.z - bbox.minPoint.z) * 0.01

        is_center = (
            abs(face_center.x - center_point.x) < tolerance_x and
            abs(face_center.y - center_point.y) < tolerance_y and
            abs(face_center.z - center_point.z) < tolerance_z
        )

        return is_center
    except Exception:
        return False

def are_faces_close(face1, face2, tolerance=1.5):
    try:
        min_distance = tolerance

        for vertex1 in face1.vertices:
            for vertex2 in face2.vertices:
                distance = vertex1.geometry.distanceTo(vertex2.geometry)
                if distance < min_distance:
                    return True

        for vertex1 in face1.vertices:
            _, param = face2.evaluator.getParameterAtPoint(vertex1.geometry)
            result, closest_point = face2.evaluator.getPointAtParameter(param)
            distance = vertex1.geometry.distanceTo(closest_point)
            if result and distance < min_distance:
                return True

        for vertex2 in face2.vertices:
            _, param = face1.evaluator.getParameterAtPoint(vertex2.geometry)
            result, closest_point = face1.evaluator.getPointAtParameter(param)
            distance = vertex2.geometry.distanceTo(closest_point)
            if result and distance < min_distance:
                return True

        return False
    except Exception:
        return False

def is_inside_face(face, body):
    try:
        u, v = 0.5, 0.5
        centroid = adsk.core.Point2D.create(u, v)
        result, world_point = face.evaluator.getPointAtParameter(centroid)
        if not result:
            return False

        result, normal = face.evaluator.getNormalAtPoint(world_point)
        if not result:
            return False

        transform = get_body_transform(body)
        world_point = transform_point(world_point, transform)
        normal = transform_vector(normal, transform)

        bbox = body.boundingBox
        body_centroid = adsk.core.Point3D.create(
            (bbox.minPoint.x + bbox.maxPoint.x) / 2,
            (bbox.minPoint.y + bbox.maxPoint.y) / 2,
            (bbox.minPoint.z + bbox.maxPoint.z) / 2
        )
        body_centroid = transform_point(body_centroid, transform)

        direction_to_centroid = adsk.core.Vector3D.create(
            body_centroid.x - world_point.x,
            body_centroid.y - world_point.y,
            body_centroid.z - world_point.z
        )
        direction_to_centroid.normalize()

        angle_to_centroid = normal.angleTo(direction_to_centroid)
        return angle_to_centroid < (math.pi / 2 + 0.01)
    except:
        return False

def get_smallest_faces(faces, max_area=200, count=100):
    filtered_faces = [face for face in faces if face.area < max_area]
    sorted_faces = sorted(filtered_faces, key=lambda face: face.area)
    return sorted_faces[:count]

class PressPullCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self, distance):
        super().__init__()
        self._distance = distance

    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            distance_input = inputs.itemById('distance')
            if distance_input:
                distance_input.value = self._distance
        except:
            pass

class PressPullCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, distance):
        super().__init__()
        self._distance = distance

    def notify(self, args):
        try:
            cmd = args.command
            on_execute = PressPullCommandExecuteHandler(self._distance)
            cmd.execute.add(on_execute)

            inputs = cmd.commandInputs
            inputs.addValueInput('distance', 'Distance', 'cm', adsk.core.ValueInput.createByReal(self._distance))
        except:
            pass

def execute_press_pull(faces, distance):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct

        if not design:
            return False

        ui.activeSelections.clear()

        for face in faces:
            ui.activeSelections.add(face)

        cmd_def = ui.commandDefinitions.itemById('FusionPressPullCommand')
        if not cmd_def:
            return False

        cmd_created_handler = PressPullCommandCreatedHandler(distance)
        cmd_def.commandCreated.add(cmd_created_handler)
        cmd_def.execute()

        return True
    except:
        return False

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct

        if not design:
            ui.messageBox('No active design', 'Error')
            return

        selected_entity = ui.selectEntity("Select a body or component", "Bodies,Occurrences")
        
        if not selected_entity:
            ui.messageBox('No entity selected', 'Error')
            return

        selected_entity = selected_entity.entity

        if isinstance(selected_entity, adsk.fusion.BRepBody):
            target_body = selected_entity
        elif isinstance(selected_entity, adsk.fusion.Occurrence):
            if selected_entity.bRepBodies.count == 0:
                ui.messageBox('The selected component has no bodies', 'Error')
                return
            target_body = selected_entity.bRepBodies.item(0)
        else:
            ui.messageBox('Selected entity is not a body or component', 'Error')
            return

        target_faces = target_body.faces
        processed_faces = set()
        all_faces_to_split = adsk.core.ObjectCollection.create()
        all_tools = adsk.core.ObjectCollection.create()

        for face in target_faces:
            if is_center_plane_face(face, target_body):
                continue
            if not is_face_oriented_inwards(face, target_body):
                continue

            face_id = face.tempId

            if face_id in processed_faces:
                continue

            all_faces_to_split.add(face)

            for occurrence in design.rootComponent.allOccurrences:
                if occurrence == selected_entity.assemblyContext:
                    continue

                for body in occurrence.bRepBodies:
                    if not body.isSolid or not body.isVisible:
                        continue

                    for tool_face in body.faces:
                        if is_face_oriented_outwards(tool_face, body):
                            if face.boundingBox.intersects(tool_face.boundingBox):
                                all_tools.add(tool_face)

        if all_tools.count == 0:
            ui.messageBox("No tools found for splitting")
        else:
            try:
                split_face_feature_input = design.rootComponent.features.splitFaceFeatures.createInput(all_faces_to_split, all_tools, True)
                try:
                    design.rootComponent.features.splitFaceFeatures.add(split_face_feature_input)
                except RuntimeError as e:
                    ui.messageBox(f"Runtime error during face split: {e}")
            except Exception as e:
                ui.messageBox(f"Error in split face creation: {e}")

        internal_faces = [face for face in target_faces if not is_center_plane_face(face, target_body) and is_inside_face(face, target_body)]

        if not internal_faces:
            ui.messageBox("No internal faces found for press-pull")
            return

        smallest_faces = get_smallest_faces(internal_faces)

        if not execute_press_pull(smallest_faces, -8.0):
            ui.messageBox("Press-pull operation failed")
        
    except Exception as e:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        else:
            print('Failed:\n{}'.format(traceback.format_exc()))
    finally:
        adsk.terminate()

run(adsk.fusion.Design.getDefault())
