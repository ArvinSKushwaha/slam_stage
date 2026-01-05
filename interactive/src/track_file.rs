#[derive(serde::Deserialize)]
pub struct TrackFile {
    pub track: std::path::PathBuf,
    pub threshold: u8,
    #[serde(default)]
    pub agents: Vec<AgentFile>,
}

#[derive(serde::Deserialize)]
pub struct AgentFile {
    pub scale: f32,
    #[serde(deserialize_with = "glam_map")]
    pub position: glam::Vec2,
    #[serde(deserialize_with = "glam_map")]
    pub heading: glam::Vec2,
    #[serde(default)]
    pub lidar: LidarFile,
}

impl Default for AgentFile {
    fn default() -> Self {
        AgentFile {
            scale: 1.0,
            position: glam::Vec2::ZERO,
            heading: glam::Vec2::X,
            lidar: Default::default(),
        }
    }
}

fn glam_map<'de, D>(d: D) -> Result<glam::Vec2, D::Error>
where
    D: serde::Deserializer<'de>,
{
    #[derive(serde::Deserialize)]
    #[serde(field_identifier, rename_all = "lowercase")]
    enum Field {
        X,
        Y,
    }

    struct GlamVec2Visitor;

    impl<'de1> serde::de::Visitor<'de1> for GlamVec2Visitor {
        type Value = glam::Vec2;

        fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
            formatter.write_str("`glam::Vec2`")
        }

        fn visit_seq<V>(self, mut seq: V) -> Result<glam::Vec2, V::Error>
        where
            V: serde::de::SeqAccess<'de1>,
        {
            let x = seq
                .next_element()?
                .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
            let y = seq
                .next_element()?
                .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
            Ok(glam::vec2(x, y))
        }

        fn visit_map<V>(self, mut map: V) -> Result<glam::Vec2, V::Error>
        where
            V: serde::de::MapAccess<'de1>,
        {
            let mut x = None;
            let mut y = None;
            while let Some(key) = map.next_key()? {
                match key {
                    Field::X => {
                        if x.is_some() {
                            return Err(serde::de::Error::duplicate_field("x"));
                        }
                        x = Some(map.next_value()?);
                    }
                    Field::Y => {
                        if y.is_some() {
                            return Err(serde::de::Error::duplicate_field("y"));
                        }
                        y = Some(map.next_value()?);
                    }
                }
            }
            let x = x.ok_or_else(|| serde::de::Error::missing_field("x"))?;
            let y = y.ok_or_else(|| serde::de::Error::missing_field("y"))?;
            Ok(glam::vec2(x, y))
        }
    }

    d.deserialize_any(GlamVec2Visitor)
}

#[derive(serde::Deserialize)]
#[serde(untagged)]
pub enum LidarFile {
    Count { count: usize },
}

impl Default for LidarFile {
    fn default() -> Self {
        Self::Count { count: 60 }
    }
}
